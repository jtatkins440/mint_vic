#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float64MultiArray.h"
#include "motion_intention/HistoryStamped.h"
#include "motion_intention/SetInt.h"
#include <sstream>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <memory>
#include <deque>
#include <chrono>
#include <mutex>
#include <thread>
#include "Iir.h" // filtering library

class HistoryHandlerWrapper{
	public:
	HistoryHandlerWrapper(){
		nh = ros::NodeHandle("history_handler");
		//nh.getParam("state_dim", state_dim); // unused, fix this!
		nh.getParam("seq_length", seq_length);
		nh.getParam("history_rate", history_rate);
		history_dt = 1.0 / history_rate;
		//history_dim = state_dim * 3;

		for (int i = 0; i < filter_dim; i++){
			filter_array[i].setup(filter_sampling_freq, filter_cutoff_freq);
		}
		
		sub_pose = nh.subscribe("/iiwa/ee_pose", 1, &HistoryHandlerWrapper::callbackPose, this);
		sub_vel = nh.subscribe("/iiwa/ee_vel", 1, &HistoryHandlerWrapper::callbackVelocity, this);
		sub_acc = nh.subscribe("/iiwa/ee_acc", 1, &HistoryHandlerWrapper::callbackAcceleration, this);
		pub_hist = nh.advertise<motion_intention::HistoryStamped>("/ee_history", 1);

		b_state_history_ready = false;
	};

	// attributes
	//int state_dim;
	int seq_length;
	double history_rate;
	double history_dt;
	//int history_dim;
	std::vector<double> current_position_vec;
	std::vector<double> current_velocity_vec;
	std::vector<double> current_acceleration_vec;
	std::vector<double> state_vector;
	std::vector<double> filtered_state_vector;
	std::vector<std::vector<double>> state_history_vector;
	bool b_state_history_ready;
	std::mutex mtx_pos; // mutex guards for the async threads updating the values
	std::mutex mtx_vel;
	std::mutex mtx_acc;

	// filter attributes, values needs to be known at compile time!
	const int filter_order {2};
	const double filter_cutoff_freq {40};
	const double filter_sampling_freq {200};
	static const int filter_dim {6};
	Iir::Butterworth::LowPass<filter_order> filter_array[filter_dim];

	// ros attributes
	ros::NodeHandle nh;
	ros::Subscriber sub_pose;
	ros::Subscriber sub_vel;
	ros::Subscriber sub_acc;
	ros::Publisher pub_hist;

	// methods
	void callbackPose(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void callbackVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg);
	void callbackAcceleration(const geometry_msgs::TwistStamped::ConstPtr& msg);

	void filterStateVector();
	void updateDeque();
	void publishHistoryArray();

	void mainLoop();
}

void HistoryHandlerWrapper::callbackPose(const geometry_msgs::PoseStamped::ConstPtr& msg){
	std::lock_guard<std::mutex> guard(mtx_pos);
    state_vector[0] = msg->pose.position.x;
	state_vector[1] = msg->pose.position.z;
	//current_position_vec.clear();
    //current_position_vec = {msg->pose.position.x, msg->pose.position.z};
};

void HistoryHandlerWrapper::callbackVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg){
	std::lock_guard<std::mutex> guard(mtx_vel);
	state_vector[2] = msg->twist.linear.x;
	state_vector[3] = msg->twist.linear.z;
	//current_velocity_vec.clear();
	//current_velocity_vec = {msg->twist.linear.x, msg->twist.linear.z};
};

void HistoryHandlerWrapper::callbackAcceleration(const geometry_msgs::TwistStamped::ConstPtr& msg){
	std::lock_guard<std::mutex> guard(mtx_acc);
	state_vector[4] = msg->twist.linear.x;
	state_vector[5] = msg->twist.linear.z;
	//current_acceleration_vec.clear();
	//current_acceleration_vec = {msg->twist.linear.x, msg->twist.linear.z};
};

void HistoryHandlerWrapper::filterStateVector(){
	filtered_state_vector.clear();
	for (int i = 0; i < filter_dim; i++){
		filtered_state_vector.push_back(filter_array[i].filter(state_vector[i]));
	}
};

void HistoryHandlerWrapper::updateDeque(){
	// 
	filterStateVector(); // updates filtered_state_vector with current filtered values
	if (seq_length <= state_history_vector.size()){
		state_history_vector.pop_front();
		b_state_history_ready = true;
	}

	state_history_vector.push_back(filtered_state_vector);
	
};

// FINISH THIS FIRST
void HistoryHandlerWrapper::publishHistoryArray(){
	if (b_state_history_ready){
		HistoryHandlerWrapper::HistoryStamped hist_msg = HistoryHandlerWrapper::HistoryStamped
	}
};

// THEN THIS
void HistoryHandlerWrapper::mainLoop(){
	// if enough time has past, update the deque then publish
};

int main(int argc, char **argv){
	HistoryHandlerWrapper history_handler;
	history_handler.mainLoop();
}