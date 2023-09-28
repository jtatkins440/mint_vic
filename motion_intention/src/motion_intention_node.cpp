#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include <sstream>
#include "mintwrapper.h"
#include <iostream>
#include <fstream>
#include <memory>
#include <deque>


class MIntNodeWrapper{
	private:
	ros::NodeHandle nh;
	std::deque<Eigen::ArrayXf> input_deque;
	float inference_rate;
	float input_sample_rate;

	//ros::Time start_time = ros::Time::now(); 
	static void subscriberCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

	public:
		//MIntNodeWrapper(){};

		MIntNodeWrapper(){
			ros::NodeHandle nh;
			//nh.getParam
			
			sub = nh.subscribe("/ee_pose", 1, subscriberCallback); // assumes it's reading a task_space PoseStamped message
			pub = nh.advertise<geometry_msgs::PoseStamped>("/ee_pose_eq", 1);
		};
	void mainLoop();
	ros::Subscriber sub;
	ros::Publisher pub;

	
	
};


void MIntNodeWrapper::subscriberCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	geometry_msgs::Point position_new;
	position_new = msg -> pose.position;
	
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

void MIntNodeWrapper::mainLoop() {
	
	ros::Rate r(500);
	while (ros::ok()){
		geometry_msgs::PoseStamped pose_s; //= geometry_msgs::PoseStamped();
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
