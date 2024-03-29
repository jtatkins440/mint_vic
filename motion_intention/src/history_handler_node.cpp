#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "motion_intention/HistoryStamped.h"
#include <motion_intention/AdmitStateStamped.h>
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
#include "iir/Iir.h" // filtering library

class HistoryHandlerWrapper{
	public:
	HistoryHandlerWrapper(){
		nh = ros::NodeHandle("history_handler");
		nh.getParam("state_dim", state_dim);
		nh.getParam("seq_length", seq_length);
		nh.getParam("history_rate", history_rate);
		history_dt = 1.0 / history_rate;
		history_dim = state_dim * 3;
		pos_vel_dim = state_dim * 2;
		spinner_thread_num = 2; // one for sub, one for pub, right?
		main_loop_duration = std::chrono::duration<double>(history_dt);

		for (int i = 0; i < filter_dim; i++){
			filter_array[i].setup(filter_sampling_freq, filter_cutoff_freq);
		}
		
		//sub_pose = nh.subscribe("/iiwa/ee_pose", 1, &HistoryHandlerWrapper::callbackPose, this);
		//sub_vel = nh.subscribe("/iiwa/ee_vel", 1, &HistoryHandlerWrapper::callbackVelocity, this);
		//sub_acc = nh.subscribe("/iiwa/ee_acc", 1, &HistoryHandlerWrapper::callbackAcceleration, this);
		sub_admit = nh.subscribe("/iiwa/admit_state", 1, &HistoryHandlerWrapper::callbackAdmitState, this);
		pub_hist = nh.advertise<motion_intention::HistoryStamped>("/ee_history", 1, true);
		//pub_nn_input = nh.advertise<motion_intention::HistoryStamped>("/ee_history", 1); // TODO: add functionality to let it publish the full input and outputs we can use to record training inputs to the neural networks during calibration trials

		b_state_history_ready = false;

		hist_msg.history.layout.dim = {std_msgs::MultiArrayDimension(), std_msgs::MultiArrayDimension()};
		// dim[0] is the vertical dimension of your matrix
        hist_msg.history.layout.dim[0].label = "pos_vel_acc";
        hist_msg.history.layout.dim[0].size = history_dim;
        hist_msg.history.layout.dim[0].stride = history_dim * seq_length;
        // dim[1] is the horizontal dimension of your matrix
        hist_msg.history.layout.dim[1].label = "samples";
        hist_msg.history.layout.dim[1].size = seq_length;
        hist_msg.history.layout.dim[1].stride = seq_length;

		std::string logstring = "history_handler: recieved state_dim: " + std::to_string(state_dim) + "\nseq_length: " + std::to_string(seq_length) + "\nhistory_rate: " + std::to_string(history_rate);
		ROS_INFO(logstring.c_str());
		pos_vel_state_vector = std::vector<double>(state_dim * 2, 0);
		state_vector = std::vector<double>(state_dim * 3, 0);
		last_state_vector = std::vector<double>(state_dim * 3, 0);
		flat_history_vector = std::vector<double>(history_dim * seq_length, 0); //should be a 125 * 6 vector of zeros
	};

	// attributes
	int state_dim;
	int pos_vel_dim;
	int seq_length;
	double history_rate;
	double history_dt;
	int history_dim;
	std::vector<double> pos_vel_state_vector;
	std::vector<double> state_vector;
	std::vector<double> last_state_vector;
	std::vector<double> filtered_state_vector;
	std::deque<std::vector<double>> state_history_vector;
	bool b_state_history_ready;
	//std::mutex mtx_pos; // mutex guards for the async threads updating the values
	//std::mutex mtx_vel;
	//std::mutex mtx_acc;
	std::chrono::duration<double> main_loop_duration;

	// filter attributes, values needs to be known at compile time! look for a workaround that doesn't involve hardcoding...
	static constexpr int filter_order {2};
	static constexpr double filter_cutoff_freq {40};
	static constexpr double filter_sampling_freq {200};
	static constexpr int filter_dim {6}; // should be 3 * state_dim in order tow work correctly
	Iir::Butterworth::LowPass<filter_order> filter_array[filter_dim];

	// ros attributes
	ros::NodeHandle nh;
	//ros::Subscriber sub_pose;
	//ros::Subscriber sub_vel;
	//ros::Subscriber sub_acc;
	ros::Subscriber sub_admit;
	ros::Publisher pub_hist;
	motion_intention::HistoryStamped hist_msg;
	std::vector<double> flat_history_vector;
	int spinner_thread_num;

	// methods
	//void callbackPose(const geometry_msgs::PoseStamped::ConstPtr& msg);
	//void callbackVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg);
	//void callbackAcceleration(const geometry_msgs::TwistStamped::ConstPtr& msg);
	void callbackAdmitState(const motion_intention::AdmitStateStamped::ConstPtr& msg);

	void filterStateVector();
	void updateDeque();
	void publishHistoryArray();

	void waitRemainingLoopTime(std::chrono::time_point<std::chrono::steady_clock> start_time, std::chrono::duration<double> desired_duration);
	bool isStateVectorUpdated();
	void mainLoop();
};

/*
void HistoryHandlerWrapper::callbackPose(const geometry_msgs::PoseStamped::ConstPtr& msg){
	//std::lock_guard<std::mutex> guard(mtx_pos);
    state_vector[0] = msg->pose.position.x;
	state_vector[1] = msg->pose.position.z;
	//current_position_vec.clear();
    //current_position_vec = {msg->pose.position.x, msg->pose.position.z};
};
*/

/*
void HistoryHandlerWrapper::callbackVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg){
	//std::lock_guard<std::mutex> guard(mtx_vel);
	state_vector[2] = msg->twist.linear.x;
	state_vector[3] = msg->twist.linear.z;
	//current_velocity_vec.clear();
	//current_velocity_vec = {msg->twist.linear.x, msg->twist.linear.z};
};
*/

/*
void HistoryHandlerWrapper::callbackAcceleration(const geometry_msgs::TwistStamped::ConstPtr& msg){
	//std::lock_guard<std::mutex> guard(mtx_acc); // should unlock when function ends
	state_vector[4] = msg->twist.linear.x;
	state_vector[5] = msg->twist.linear.z;
	//current_acceleration_vec.clear();
	//current_acceleration_vec = {msg->twist.linear.x, msg->twist.linear.z};
};
*/

void HistoryHandlerWrapper::callbackAdmitState(const motion_intention::AdmitStateStamped::ConstPtr& msg){
	//std::lock_guard<std::mutex> guard(mtx_acc); // should unlock when function ends
	state_vector[0] = msg->pose.position.x;
	state_vector[1] = msg->pose.position.z;
	state_vector[2] = msg->twist.linear.x;
	state_vector[3] = msg->twist.linear.z;
	state_vector[4] = msg->accel.linear.x;
	state_vector[5] = msg->accel.linear.z;
	//current_acceleration_vec.clear();
	//current_acceleration_vec = {msg->twist.linear.x, msg->twist.linear.z};
};

bool HistoryHandlerWrapper::isStateVectorUpdated(){
	for (int i = 0; i < state_vector.size(); i++){
		if (state_vector[i] == last_state_vector[i]){
			return false;
		}
	}
	last_state_vector.clear();
	last_state_vector = state_vector;
	return true; // only happens if none of the elements in state_vector are the same as last_state_vector
};

void HistoryHandlerWrapper::filterStateVector(){
	/*
	std::string logstring = "history_handler: state_vector: \n";
	for (int i = 0; i < state_vector.size(); i++){
		logstring = logstring + " " + std::to_string(state_vector[i]);
	}
	ROS_INFO(logstring.c_str());
	*/
	filtered_state_vector.clear();
	for (int i = 0; i < filter_dim; i++){
		filtered_state_vector.push_back(filter_array[i].filter(state_vector[i]));
		//filtered_state_vector.push_back(state_vector[i]);
	}
	
	/*
	logstring = "history_handler: filtered_state_vector: \n";
	for (int i = 0; i < filtered_state_vector.size(); i++){
		logstring = logstring + " " + std::to_string(filtered_state_vector[i]);
	}
	ROS_INFO(logstring.c_str());
	*/

	pos_vel_state_vector.clear();
	for (int i = 0; i < pos_vel_dim; i++){
		pos_vel_state_vector.push_back(filtered_state_vector[i]);
	}
};

void HistoryHandlerWrapper::updateDeque(){
	if (isStateVectorUpdated()){
		filterStateVector(); // updates filtered_state_vector with current filtered values
		/*
		if (seq_length == state_history_vector.size()){
			state_history_vector.pop_front();
			b_state_history_ready = true;
		}
		else if (seq_length < state_history_vector.size()){ // this case shouldn't happen but just in case we should make sure there's only seq_length elements in there
			while (seq_length < state_history_vector.size()){
				state_history_vector.pop_front();
			}
			b_state_history_ready = true;
		}
		*/

		if (seq_length <= state_history_vector.size()){
			state_history_vector.pop_front();
			b_state_history_ready = true;
		}
		else {
			b_state_history_ready = false; // it's defaulted to false but it's probably good to make sure just in case
		}

		state_history_vector.push_back(filtered_state_vector);
	}
};

void HistoryHandlerWrapper::publishHistoryArray(){
		// update the header's timestamp and the current state
		hist_msg.header.stamp = ros::Time::now(); 
		hist_msg.state = pos_vel_state_vector; // .data() ?

		// clear the only history array and put the new one in there
		hist_msg.history.data.clear();
		//flat_history_vector.clear(); // does clearing change the size? if so, don't do this
		for (int i = 0; i < history_dim; i++){
			for (int j = 0; j < seq_length; j++){
				flat_history_vector[i * seq_length + j] = state_history_vector[j][i]; // should overwrite every element quickly, want to fill it in 'row by row'
			}
		}
		hist_msg.history.data = flat_history_vector;
		
		/*std::string logstring = "history_handler: flat_history_vector: \n";
		for (int i = 0; i < flat_history_vector.size(); i++){
			logstring = logstring + " " + std::to_string(flat_history_vector[i]);
		}
		ROS_INFO(logstring.c_str());
		*/
		pub_hist.publish(hist_msg);
};

void HistoryHandlerWrapper::waitRemainingLoopTime(std::chrono::time_point<std::chrono::steady_clock> start_time, std::chrono::duration<double> desired_duration){
	std::chrono::duration<double> diff = std::chrono::steady_clock::now() - start_time; 
	while (diff.count() < desired_duration.count()){
		diff = std::chrono::steady_clock::now() - start_time;
		
	}
};

// THEN THIS
void HistoryHandlerWrapper::mainLoop(){
	// if enough time has past, update the deque then publish
	ros::AsyncSpinner spinner(spinner_thread_num);
	spinner.start();

	std::chrono::time_point<std::chrono::steady_clock> start_time = std::chrono::steady_clock::now();
	waitRemainingLoopTime(start_time, main_loop_duration);

	ROS_INFO("history_handler: Starting main loop...");
	while (ros::ok()){
		start_time = std::chrono::steady_clock::now();
		updateDeque();
		if (b_state_history_ready){
			publishHistoryArray();
		}
		waitRemainingLoopTime(start_time, main_loop_duration);

	}
	spinner.stop();
	ros::waitForShutdown();
};

int main(int argc, char **argv){
	ros::init(argc, argv, "history_handler");
	HistoryHandlerWrapper history_handler;
	history_handler.mainLoop();
	return 0;
}