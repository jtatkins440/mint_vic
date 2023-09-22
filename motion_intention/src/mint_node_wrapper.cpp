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
			
			ros::Subscriber sub = n.subscribe("sub_topic_name", 1, subscriberCallback);
			ros::Publisher chatter_pub = n.advertise<std_msgs::String>("pub_topic_name", 1);
		};


	private:
	std::deque<Eigen::ArrayXf> test_deque;
	void subscriberCallback();  
};

void MIntNodeWrapper::subscriberCallback()
{
	int max_deque_size = 10;
	for (int i = 0; i < 100; i++)
	{
		if (test_deque.size() >= max_deque_size) {test_deque.pop_front();}
		test_deque.push_back(i);
		if (i % 10 == 0) { for (int n : test_deque) {std::cout << n << ", ";} std::cout << std::endl;}
	}
	for (int n : test_deque) {std::cout << n << ", ";} std::cout << std::endl;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "node_name");
}
