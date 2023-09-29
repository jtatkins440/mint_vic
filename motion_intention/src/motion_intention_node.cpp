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

class MIntNodeWrapper{
	public:
		//MIntNodeWrapper(){};

		

		MIntNodeWrapper(){
			ros::NodeHandle nh("mint");
			//nh.getParam
			
			sub = nh.subscribe("/ee_pose", 1, subscriberCallback); // assumes it's reading a task_space PoseStamped message
			pub = nh.advertise<geometry_msgs::PoseStamped>("/ee_pose_eq", 1);

			/*
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

			ros::param::param<double>("sample_time", sample_time, 0.005);
			ros::param::param<double>("allowable_time", allowable_time_tolerance, 0.0005);
			ros::param::param<int>("inference_rate", inference_rate, 500);
			ros::param::param<int>("pose_deque_min_size", pose_deque_min_size, 3);
			*/

			
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
			//ros::param::param<int>("position_size", position_size, 3);

			//startDequeHandler();
		};

		void mainLoop();
		void dequeHandler();
		//void startDequeHandler();

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
		//MIntWrapper mintnet;

		//ros::Time start_time = ros::Time::now(); 
		static void subscriberCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
		void updateDeque(Eigen::ArrayXf new_pose, float dt);

		//std::thread dequeHandlerThread;

	private:
		std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();
		//std::mutex mtx;
};

//void MIntNodeWrapper::startDequeHandler(){
//	std::thread dequeHandlerThread (&MIntNodeWrapper::dequeHandler, this); 
//	dequeHandlerThread.detach();
//	return;
//};

void MIntNodeWrapper::subscriberCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	geometry_msgs::Point position_new;
	position_new = msg -> pose.position;
	
	Eigen::Matrix<float, 3, 1> current_position;
	current_position << position_new.x, position_new.y, position_new.z;

	return;
};

void MIntNodeWrapper::dequeHandler(){
	//std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point time_current = std::chrono::steady_clock::now();
	bool update_deque_flag = true;

	//while (update_deque_flag){
	time_current = std::chrono::steady_clock::now();
	std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(time_current - time_start);
	if (sample_time <= time_span.count()){
		// if enough time has passed, update the deque and refit the spine
		//Eigen::ArrayXf pose_new(mintnet.output_chn_size, 1);
		Eigen::Array<float, 2, 1> pose_new;
		pose_new << current_position(0), current_position(1); // x and y position only!
		std::cout << "Before updateDeque..." << std::endl;
		updateDeque(pose_new, time_span.count());
		std::cout << "After updateDeque..." << std::endl;
		if (b_deque_ready){
			//mtx.lock();
			std::cout << "Before mintnet.fit(pose_new, input_deque);..." << std::endl;
			mintnet.fit(pose_new, input_deque);
			std::cout << "After mintnet.fit(pose_new, input_deque);..." << std::endl;
			//mtx.unlock();
		}
		std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();
	}
		//else {
		//	dequeHandlerThread
		//}
	//}
};

/*
void MIntNodeWrapper::subscriberCallback(){
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

	int max_deque_size = 10;
	for (int i = 0; i < 100; i++)
	{
		if (test_deque.size() >= max_deque_size) {test_deque.pop_front();}
		test_deque.push_back(i);
		if (i % 10 == 0) { for (int n : test_deque) {std::cout << n << ", ";} std::cout << std::endl;}
	}
	for (int n : test_deque) {std::cout << n << ", ";} std::cout << std::endl;
	
	return;
};
*/

void MIntNodeWrapper::updateDeque(Eigen::ArrayXf new_pose, float dt){
	// WIP, have different behavior for different motion intention methods
	std::cout << "b_deque_ready: " << b_deque_ready << std::endl;
	std::cout << "input_deque.size(): " << input_deque.size() << std::endl;

	// if mintnet

	// fill in deque
	//pose_deque.push_back(new_pose);

	//std::cout << "Before pose_deque.size()" << std::endl;
	if (pose_deque.size() >= input_deque_seq_length) {
		pose_deque.pop_front(); // if we don't seperate these two conditions then pose_deque would be longer than input_deque. Not a big problem though, merge later if needed.
	}

	//std::cout << "Before input_deque.size()" << std::endl;
	std::cout << "mintnet.input_seq_length: " << mintnet.input_seq_length << std::endl;
	std::cout << "input_deque_seq_length: " << input_deque_seq_length << std::endl;
	std::cout << "(input_deque.size() >= mintnet.input_seq_length): " << (input_deque.size() >= mintnet.input_seq_length) << std::endl;
	if (input_deque.size() >= input_deque_seq_length) {
		input_deque.pop_front();
		b_deque_ready = true;
	}

	//std::cout << "Before pose_deque.push_back" << std::endl;
	pose_deque.push_back(new_pose);
	if (pose_deque.size() > pose_deque_min_size){
		//std::cout << "Before vel_est" << std::endl;
		Eigen::Array<float, 2, 1> vel_est = (pose_deque[pose_deque.size() - 1] - pose_deque[pose_deque.size() - 2]) / dt;
		Eigen::Array<float, 2, 1> acc_est = (vel_est - (pose_deque[pose_deque.size() - 2] - pose_deque[pose_deque.size() - 3]) / dt) / dt;
		//std::cout << "Before new_input" << std::endl;
		Eigen::Array<float, 4, 1> new_input;
		//std::cout << "vel_est: " << vel_est << " acc_est: " << acc_est << std::endl;
		new_input << vel_est, acc_est;
		//std::cout << "new_input: " << new_input << std::endl;
		//std::cout << "Before input_deque.push_back(new_input);" << std::endl;
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

		dequeHandler();
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
	//minty.startDequeHandler();
	std::cout << "DequeHandler started!" << std::endl;
	//std::thread dequeHandlerThread (&MIntNodeWrapper::dequeHandler, this); 
	minty.mainLoop();
}
