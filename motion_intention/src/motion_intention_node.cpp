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

class MIntNodeWrapper{
	public:
		//MIntNodeWrapper(){};

		MIntNodeWrapper(){
			ros::NodeHandle nh("mint");
			//nh.getParam
			
			sub = nh.subscribe("/ee_pose", 1, subscriberCallback); // assumes it's reading a task_space PoseStamped message
			pub = nh.advertise<geometry_msgs::PoseStamped>("/ee_pose_eq", 1);

			if (ros::param::get("model_weights_path", model_weights_path)) {
				std::cout << "Recieved model_weights_path as " << model_weights_path << std::endl;
			}
			else {
				ROS_INFO("!!!No model_weights_path in motion_intention_node!!!");
			};

			if (ros::param::get("hparam_json_path", hyperparam_weights_path)) {
				std::cout << "Recieved hyperparam_weights_path as " << hyperparam_weights_path << std::endl;
			}
			else {
				ROS_INFO("!!!No hyperparam_weights_path in motion_intention_node!!!");
			};

			MIntWrapper mintnet(model_weights_path, hyperparam_weights_path);

			ros::param::param<float>("sample_time", sample_time, 0.005);
			ros::param::param<float>("allowable_time", allowable_time_tolerance, 0.0005);
			ros::param::param<int>("inference_rate", inference_rate, 500);
			ros::param::param<int>("pose_deque_min_size", pose_deque_min_size, 3);

		};
		void mainLoop();
		ros::Subscriber sub;
		ros::Publisher pub;
		MIntWrapper mintnet;
		
		ros::NodeHandle nh;
		std::deque<Eigen::ArrayXf> input_deque; // deque of inputs (vel and acc) to the MIntNet
		std::deque<Eigen::ArrayXf> pose_deque; // deque of past states (position/orientation) of same size used for other fitting methods
		int pose_deque_min_size;
		float sample_time;
		float allowable_time_tolerance;
		int inference_rate;
		bool b_deque_ready = false; // flag for toggling if deque is full or not
		bool b_mint_ready = false; // flag for toggling if mint method is fitted and ready to give predictions

		std::string model_weights_path, hyperparam_weights_path;
		//MIntWrapper mintnet;
		auto start{std::chrono::steady_clock::now()};

		//ros::Time start_time = ros::Time::now(); 
		static void subscriberCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
		void updateDeque(Eigen::ArrayXf new_pose, float dt);

	private:
		
};


void MIntNodeWrapper::subscriberCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	geometry_msgs::Point position_new;
	position_new = msg -> pose.position;
	

	auto current{std::chrono::steady_clock::now()};
	
	std::chrono::duration<double> elapsed_seconds{current - start};


	if ((sample_time - allowable_time_tolerance) <= elapsed_seconds) && (elapsed_seconds <= (sample_time + allowable_time_tolerance)) {
		// if in correct time window, restart clock, update deque, and refit the spline
		auto start{std::chrono::steady_clock::now()};
		Eigen::ArrayXf pose_new(mintnet.output_chn_size, 1) << msg.x, msg.y; //output channel size is the size of the state before differentiation
		updateDeque(pose_new, (float) elapsed_seconds);
		if b_deque_ready{
			mintnet.fit(input_deque);
		}
	}
	else if ((sample_time + allowable_time_tolerance) < elapsed_seconds){
		// if too much time has passed, restart clock but don't update the deque
		auto start{std::chrono::steady_clock::now()};
	}
	else {
		// if not enough time has passed, don't do anything
		return;
	};

	/*
	int max_deque_size = 10;
	for (int i = 0; i < 100; i++)
	{
		if (test_deque.size() >= max_deque_size) {test_deque.pop_front();}
		test_deque.push_back(i);
		if (i % 10 == 0) { for (int n : test_deque) {std::cout << n << ", ";} std::cout << std::endl;}
	}
	for (int n : test_deque) {std::cout << n << ", ";} std::cout << std::endl;
	*/
	return;
};

void MIntNodeWrapper::updateDeque(Eigen::ArrayXf new_pose, float dt){
	// WIP, have different behavior for different motion intention methods

	// if mintnet

	// fill in deque

	pose_deque.push_back(new_pose);

	if (pose_deque.size() >= mintnet.input_seq_length) {
		pose_deque.pop_front(); // if we don't seperate these two conditions then pose_deque would be longer than input_deque. Not a big problem though, merge later if needed.
	}

	if (input_deque.size() >= mintnet.input_seq_length) {
		input_deque.pop_front();
		b_deque_ready = true;
	}

	pose_deque.push_back(new_pose);
	if (pose_deque.size() > pose_deque_min_size){
		Eigen::ArrayXf vel_est = (pose_deque[pose_deque.size() - 1] - pose_deque[pose_deque.size() - 2]) / dt;
		Eigen::ArrayXf acc_est = (vel_est - (pose_deque[pose_deque.size() - 2] - pose_deque[pose_deque.size() - 3]) / dt) / dt;
		Eigen::ArrayXf new_input(mintnet.input_chn_size, 1);
		new_input << vel_est, acc_est;
		input_deque.push_back(new_input);
	}
	
	// if circlefit

	// if linearfit

	return; 
};

void MIntNodeWrapper::mainLoop() {
	
	ros::Rate r(inference_rate);
	while (ros::ok()){
		geometry_msgs::PoseStamped pose_s; // empty posestamped message


		if (b_mint_ready){
			// update empty message with fitted value
			Eigen::ArrayXf eq_pose = mintnet.getEquilibriumPoint();
			pose_s.pose.position.x = eq_pose(0);
			pose_s.pose.position.y = eq_pose(1);
		}

		pub.publish(pose_s);


		ros::spinOnce();
		r.sleep();
	}
	return;	
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "motion_intent");
	MIntNodeWrapper minty;
	minty.mainLoop();
}
