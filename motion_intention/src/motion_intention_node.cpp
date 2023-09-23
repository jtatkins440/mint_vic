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
	public:
		MIntNodeWrapper(){};

		MIntNodeWrapper(){
			ros::NodeHandle n;
			
			ros::Subscriber sub = n.subscribe("ee_pose", 1, subscriberCallback); // assumes it's reading a task_space PoseStamped message
			ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("ee_pose_eq", 1);
		};


	private:
	std::deque<Eigen::ArrayXf> input_deque;
	//ros::Time start_time = ros::Time::now(); 
	void subscriberCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);  
};


void MIntNodeWrapper::subscriberCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	position_new = msg -> position;

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
};

void MIntNodeWrapper::mainLoop() {
	while ros::ok(){
		auto pose_s = PoseStamped();
		chatter_pub.publish(pose_s);
	}
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "motion_intent");

	MIntNodeWrapper minty();
	minty.mainLoop();
}
