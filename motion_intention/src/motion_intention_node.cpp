#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "motion_intention/HistoryStamped.h"
#include "motion_intention/SetInt.h"
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
			ros::NodeHandle nh("intention_predictor");
			//nh.getParam
			
			//sub = nh.subscribe("/ee_pose", 2, &MIntNodeWrapper::subscriberCallback, this); // assumes it's reading a task_space PoseStamped message
			sub_hist = nh.subscribe("/ee_history", 2, &MIntNodeWrapper::subscriberHistoryCallback, this); // assumes it's reading a task_space PoseStamped message
			pub = nh.advertise<geometry_msgs::PoseStamped>("/ee_pose_eq", 2);

			
			if (nh.getParam("model_weights_path", model_weights_path)) {
				std::cout << "Recieved model_weights_path as " << model_weights_path << std::endl;
			}
			else {
				ROS_INFO("!!!No model_weights_path in motion_intention_node!!!");
			};

			if (nh.getParam("hparam_json_path", hyperparam_weights_path)) {
				std::cout << "Recieved hyperparam_weights_path as " << hyperparam_weights_path << std::endl;
			}
			else {
				ROS_INFO("!!!No hyperparam_weights_path in motion_intention_node!!!");
			};

			mintnet = MIntWrapper(model_weights_path, hyperparam_weights_path);
			//LineFitWrapper mint_line = LineFitWrapper(hyperparam_weights_path);
    		//CircleFitWrapper mint_circ = CircleFitWrapper(hyperparam_weights_path);

			mint_line = LineFitWrapper(hyperparam_weights_path);
    		mint_circ = CircleFitWrapper(hyperparam_weights_path);

			nh.param<double>("sample_time", sample_time, 0.005);
			nh.param<double>("allowable_time", allowable_time_tolerance, 0.0005);
			nh.param<int>("inference_rate", inference_rate, 500);
			nh.param<int>("pose_deque_min_size", pose_deque_min_size, 3);
			nh.param<int>("state_dim", state_dim, 2);
			nh.param<int>("seq_length", seq_length, 125);
			nh.param<int>("mint_state_dim", mint_state_dim, 4);
			nh.param<int>("cfit_state_dim", cfit_state_dim, 2);

			input_deque_seq_length = mintnet.input_seq_length;
			time_start = std::chrono::steady_clock::now();

			fit_type = 0; // use mintnet by default, update it via service.
			srv_set_mint_type = nh.advertiseService("/mint/set_motion_intention_type", &MIntNodeWrapper::setMotionIntentType, this);

			//Eigen::Array<float, 2 * state_dim, seq_length> input_array;
			//mint_state_dim = 4;

			ROS_INFO("mint: MIntNodeWrapper initalized.");
		};

		void mainLoop();
		void dequeHandler();
		void startDequeHandler();
		void fitHandler();

		//ros::Subscriber sub;
		ros::Subscriber sub_hist;
		ros::Publisher pub;
		MIntWrapper mintnet;
		LineFitWrapper mint_line;
		CircleFitWrapper mint_circ;
		int fit_type; // 0 for mintnet, 1 for line, 2 for circle.

		ros::NodeHandle nh;
		std::deque<Eigen::ArrayXf> input_deque; // deque of inputs (vel and acc) to the MIntNet
		std::deque<Eigen::ArrayXf> pose_deque; // deque of past states (position/orientation) of same size used for other fitting methods
		int input_deque_seq_length;
		Eigen::Matrix<float, 3, 1> current_position; // I hate hardcoding this but the static restriction on the callback is making it hard not to. Find a fix when there's time.
		Eigen::Matrix<float, 4, 1> current_state; // [pos, vel]
		Eigen::ArrayXXf mint_input_array;
		Eigen::ArrayXXf cfit_input_array;
		int pose_deque_min_size;
		double sample_time;
		double allowable_time_tolerance;
		int inference_rate;
		int position_size;
		int state_dim;
		int seq_length;
		int mint_state_dim;
		int cfit_state_dim;

		bool b_deque_ready = false; // flag for toggling if deque is full or not
		bool b_mint_ready = false; // flag for toggling if mint method is fitted and ready to give predictions


		std::string model_weights_path, hyperparam_weights_path;
		ros::ServiceServer srv_set_mint_type;
		

		void subscriberCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
		void subscriberHistoryCallback(const motion_intention::HistoryStamped::ConstPtr& msg);
		void updateDeque(Eigen::ArrayXf new_pose, float dt);
		bool setMotionIntentType(motion_intention::SetInt::Request &req, motion_intention::SetInt::Response &res);

	private:
		std::chrono::steady_clock::time_point time_start;
};

bool MIntNodeWrapper::setMotionIntentType(motion_intention::SetInt::Request &req, motion_intention::SetInt::Response &res){
	
    
    std::string out_string;
	std::string type_name;
	if (req.data == 0){
		type_name = "MIntNet";
	}
	else if (req.data == 1){
		type_name = "Line";
	}
	else if (req.data == 2){
		type_name = "Circle";
	}
	else{
		type_name = "Invalid";
		res.success = false;
		out_string = "!!!Invalid type!!! Expected 0 : MIntent, 1 : Line, or 2 : Circle";
		res.message = out_string;
		return res.success; // gets out of 
	}

	
	res.success = true;
	fit_type = req.data;
    out_string = "motion_intention class fitting type set to " + type_name + "!!!";
    res.message = out_string;
    return res.success;
};

void MIntNodeWrapper::subscriberCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	geometry_msgs::Point position_new;
	position_new = msg -> pose.position;
	
	Eigen::Matrix<float, 3, 1> current_position_temp;
	current_position_temp << position_new.x, position_new.y, position_new.z;

	current_position.swap(current_position_temp);

	return;
};

void MIntNodeWrapper::subscriberHistoryCallback(const motion_intention::HistoryStamped::ConstPtr& msg){
	int rows = msg->history.layout.dim[0].size;
	int cols = msg->history.layout.dim[1].size;
	
	std::vector<double> data = msg->history.data;
  	Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> hist_matrix(data.data(), rows, cols);

	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> full_hist_matrix = (hist_matrix.cast <float> ());
	cfit_input_array = full_hist_matrix.block(0,0,cfit_state_dim,cols).eval();
	mint_input_array = full_hist_matrix.block(2,0,mint_state_dim,cols).eval();

	

	Eigen::Matrix<float, 4, 1> current_state_temp;
	current_state_temp << (float) msg->state[0], (float) msg->state[1], (float) msg->state[2], (float) msg->state[3];
	current_state.swap(current_state_temp);
	b_deque_ready = true;

	//std::cout << "mint_input_array = " << std::endl << mint_input_array << std::endl;
	//std::cout << "cfit_input_array = " << std::endl << cfit_input_array << std::endl;
	//std::cout << "current_state = " << std::endl << current_state << std::endl;
};

void MIntNodeWrapper::fitHandler(){
	std::chrono::steady_clock::time_point time_current = std::chrono::steady_clock::now();

	time_current = std::chrono::steady_clock::now();
	std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(time_current - time_start);
	if (sample_time <= time_span.count()){
		// if enough time has passed, update the deque and refit the spine
		//std::cout << "Updating mintnet, time_span.count(): " << time_span.count() << std::endl;
		if (b_deque_ready){
			//ROS_WARN("MInt: in fitHandler, in b_deque_ready, before .fit() functions near 186:");
			//Eigen::Array<float, 4, 1> current_state;
			//Eigen::Array<float, 4, 1> current_vel_acc;
			//current_vel_acc = input_array[input_array.size() - 1];
			//current_state << current_position(0), current_position(1), current_vel_acc(0), current_vel_acc(1);
			//std::cout << "dequeHandler, current_state: " << current_state << std::endl;
			//ROS_WARN("MInt: in fitHandler, in b_deque_ready, before mintnet.fit():");
			mintnet.fit(current_state, mint_input_array); // expects [current_position; current_velocity] for first argument!
			//ROS_WARN("MInt: in fitHandler, in b_deque_ready, before mint_line.fit():");
			mint_line.fit(current_state, cfit_input_array); 
			//ROS_WARN("MInt: in fitHandler, in b_deque_ready, before mint_circ.fit():");
			mint_circ.fit(current_state, cfit_input_array); 
			b_mint_ready = true;
		}
		time_start = std::chrono::steady_clock::now();
	}
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

	//static tf2_ros::TransformBroadcaster br;

	/*
	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.frame_id = "world";
	transformStamped.child_frame_id = "ee_eq";
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.transform.rotation.x = 0.0;
	transformStamped.transform.rotation.y = 0.0;
	transformStamped.transform.rotation.z = 0.0;
	transformStamped.transform.rotation.w = 1.0;
	*/
	
	//ROS_INFO("MInt: before while (ros::ok()):");
	while (ros::ok()){
		geometry_msgs::PoseStamped pose_s; // empty posestamped message

		//dequeHandler();
		//ROS_INFO("MInt: before fitHandler():");
		//ROS_WARN("MInt: before fitHandler():");
		fitHandler();
		//Eigen::ArrayXf eq_pose = Eigen::ArrayXf::Zero(2,1);
		Eigen::ArrayXf eq_pose = Eigen::ArrayXf::Zero(2,1);
		if (b_mint_ready){
			// update empty message with fitted value
			//ROS_WARN("MInt: before if(fit_type):");
			if (fit_type == 0){
				eq_pose = mintnet.getEquilibriumPoint();
			}
			else if (fit_type == 1){
				eq_pose = mint_line.getEquilibriumPoint();
			}
			else if (fit_type == 2){
				eq_pose = mint_circ.getEquilibriumPoint();
			}
			else{
				ROS_WARN("MInt: Invalid fit_type!");
			}
			
			std::cout << "MInt: eq_pose: " << eq_pose << " for fit_type: " << fit_type << "." << std::endl;
			//ROS_WARN("MInt: before setting pose_s:");
			pose_s.pose.position.x = eq_pose(0);
			pose_s.pose.position.z = eq_pose(1);
			pose_s.pose.orientation.w = 1.0;
			pose_s.header.frame_id = "ee_eq";
			pose_s.header.stamp = ros::Time::now();
			//std::cout << "Got new eq_pose! It's: "<< eq_pose << std::endl;

		}
		//ROS_INFO("MInt: before publishing pose_s:");
		pub.publish(pose_s);
		//br.sendTransform(transformStamped);

		ros::spinOnce();
		r.sleep();
	}
	return;	
};

int main(int argc, char **argv){
	ros::init(argc, argv, "motion_intent");
	MIntNodeWrapper minty;

	minty.mainLoop();
	//deque_handler_thread.join();
}
