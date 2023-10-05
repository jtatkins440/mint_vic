#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include <sstream>
#include "mintwrapper.h"
#include <iostream>
#include <fstream>
#include <memory>
#include <deque>
#include <chrono>
#include <mutex>
#include <thread>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class MIntNodeWrapper{
	public:

		MIntNodeWrapper(){
			ros::NodeHandle nh("mint");
			//nh.getParam
			
			sub = nh.subscribe("/ee_pose", 2, &MIntNodeWrapper::subscriberCallback, this); // assumes it's reading a task_space PoseStamped message
			pub = nh.advertise<geometry_msgs::PoseStamped>("/ee_pose_eq", 2);

			
			if (ros::param::get("/mint/model_weights_path", model_weights_path)) {
				std::cout << "Recieved model_weights_path as " << model_weights_path << std::endl;
			}
			else {
				ROS_INFO("!!!No model_weights_path in motion_intention_node!!!");
			};

			if (ros::param::get("/mint/hparam_json_path", hyperparam_weights_path)) {
				std::cout << "Recieved hyperparam_weights_path as " << hyperparam_weights_path << std::endl;
			}
			else {
				ROS_INFO("!!!No hyperparam_weights_path in motion_intention_node!!!");
			};

			mintnet = MIntWrapper(model_weights_path, hyperparam_weights_path);

			ros::param::param<double>("/mint/sample_time", sample_time, 0.005);
			ros::param::param<double>("/mint/allowable_time", allowable_time_tolerance, 0.0005);
			ros::param::param<int>("/mint/inference_rate", inference_rate, 500);
			ros::param::param<int>("/mint/pose_deque_min_size", pose_deque_min_size, 3);
			
			input_deque_seq_length = mintnet.input_seq_length;
			time_start = std::chrono::steady_clock::now();

		};

		void mainLoop();
		void dequeHandler();
		void startDequeHandler();

		ros::Subscriber sub;
		ros::Publisher pub;
		MIntWrapper mintnet;

		ros::NodeHandle nh;
		std::deque<Eigen::ArrayXf> input_deque; // deque of inputs (vel and acc) to the MIntNet
		std::deque<Eigen::ArrayXf> pose_deque; // deque of past states (position/orientation) of same size used for other fitting methods
		int input_deque_seq_length;
		Eigen::Matrix<float, 3, 1> current_position; // I hate hardcoding this but the static restriction on the callback is making it hard not to. Find a fix when there's time.
		int pose_deque_min_size;
		double sample_time;
		double allowable_time_tolerance;
		int inference_rate;
		int position_size;

		bool b_deque_ready = false; // flag for toggling if deque is full or not
		bool b_mint_ready = false; // flag for toggling if mint method is fitted and ready to give predictions


		std::string model_weights_path, hyperparam_weights_path;

		void subscriberCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
		void updateDeque(Eigen::ArrayXf new_pose, float dt);

	private:
		std::chrono::steady_clock::time_point time_start;
};



void MIntNodeWrapper::subscriberCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	geometry_msgs::Point position_new;
	position_new = msg -> pose.position;
	
	Eigen::Matrix<float, 3, 1> current_position_temp;
	current_position_temp << position_new.x, position_new.y, position_new.z;

	current_position.swap(current_position_temp);

	return;
};

void MIntNodeWrapper::dequeHandler(){
	std::chrono::steady_clock::time_point time_current = std::chrono::steady_clock::now();
	bool update_deque_flag = true;


	time_current = std::chrono::steady_clock::now();
	std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(time_current - time_start);
	if (sample_time <= time_span.count()){
		// if enough time has passed, update the deque and refit the spine
		//Eigen::ArrayXf pose_new(mintnet.output_chn_size, 1);
		Eigen::Array<float, 2, 1> pose_new;
		pose_new << current_position(0), current_position(1); // x and y position only!
		//std::cout << "Before updateDeque..." << std::endl;
		std::cout << "time_span.count(): " << time_span.count() << std::endl;
		updateDeque(pose_new, time_span.count());
		//std::cout << "After updateDeque..." << std::endl;
		if (b_deque_ready){
			Eigen::Array<float, 4, 1> current_state;
			Eigen::Array<float, 4, 1> current_vel_acc;
			current_vel_acc = input_deque[input_deque.size() - 1];
			current_state << current_position(0), current_position(1), current_vel_acc(0), current_vel_acc(1);
			//std::cout << "dequeHandler, current_state: " << current_state << std::endl;

			std::cout << std::endl;
			mintnet.fit(current_state, input_deque); // expects [current_position; current_velocity] for first argument!

			b_mint_ready = true;
		}
		time_start = std::chrono::steady_clock::now();
	}
};

// this assumes the deque will be given inputs of global positions. Ideally we'd just have velocity and acceleration inputs from the admittance controller.
void MIntNodeWrapper::updateDeque(Eigen::ArrayXf new_pose, float dt){
	// if mintnet

	// fill in deque


	//std::cout << "Before pose_deque.size()" << std::endl;
	if (pose_deque.size() >= input_deque_seq_length) {
		pose_deque.pop_front(); // if we don't seperate these two conditions then pose_deque would be longer than input_deque. Not a big problem though, merge later if needed.
	}

	if (input_deque.size() >= input_deque_seq_length) {
		input_deque.pop_front();
		b_deque_ready = true;
	}


	pose_deque.push_back(new_pose);
	if (pose_deque.size() > pose_deque_min_size){
		//std::cout << "pose_deque[pose_deque.size() - 1]: " << pose_deque[pose_deque.size() - 1] << std::endl;
		//std::cout << "pose_deque[pose_deque.size() - 2]: " << pose_deque[pose_deque.size() - 2] << std::endl;
		Eigen::Array<float, 2, 1> vel_est;
		vel_est << (pose_deque[pose_deque.size() - 1] - pose_deque[pose_deque.size() - 2]) / dt;
		Eigen::Array<float, 2, 1> acc_est;
		acc_est << (vel_est - (pose_deque[pose_deque.size() - 2] - pose_deque[pose_deque.size() - 3]) / dt) / dt;
		Eigen::Array<float, 4, 1> new_input;
		//std::cout << "vel_est: " << vel_est << " acc_est: " << acc_est << std::endl;
		new_input << vel_est, acc_est;
		//std::cout << "new_input: " << new_input << std::endl;
		input_deque.push_back(new_input);
	}
	
	// if circlefit

	// if linearfit

	return; 
};

void MIntNodeWrapper::mainLoop() {
	
	ros::Rate r(inference_rate);

	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.frame_id = "world";
	transformStamped.child_frame_id = "ee_eq";
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.transform.rotation.x = 0.0;
	transformStamped.transform.rotation.y = 0.0;
	transformStamped.transform.rotation.z = 0.0;
	transformStamped.transform.rotation.w = 1.0;

	

	while (ros::ok()){
		geometry_msgs::PoseStamped pose_s; // empty posestamped message

		dequeHandler();
		if (b_mint_ready){
			// update empty message with fitted value
			Eigen::ArrayXf eq_pose = mintnet.getEquilibriumPoint();
			pose_s.pose.position.x = eq_pose(0);
			pose_s.pose.position.y = eq_pose(1);
			pose_s.pose.orientation.w = 1.0;
			pose_s.header.frame_id = "ee_eq";
			std::cout << "Got new eq_pose! It's: "<< eq_pose << std::endl;

			transformStamped.header.stamp = ros::Time::now();
			transformStamped.transform.translation.x = eq_pose(0);
			transformStamped.transform.translation.x = eq_pose(1);
		}

		pub.publish(pose_s);
		br.sendTransform(transformStamped);

		ros::spinOnce();
		r.sleep();
	}
	return;	
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "motion_intent");
	MIntNodeWrapper minty;
	//minty.startDequeHandler();
	//std::cout << "DequeHandler started!" << std::endl;
	//std::thread dequeHandlerThread (&MIntNodeWrapper::dequeHandler, this); 
	//std::thread deque_handler_thread(&MIntNodeWrapper::dequeHandler, minty);
	minty.mainLoop();
	//deque_handler_thread.join();
}
