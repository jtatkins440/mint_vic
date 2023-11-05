#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <string.h>
#include "UdpServer.h"

using namespace std;

const std::string udp_addr_gui("192.168.0.102");
const int udp_port_gui = 50000;

double gui_data[40];
double targetXYold[2];
double targetXY[2];
double endEffectorXY[2];
double targets[6][2];

void targetXYoldCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    targetXYold[0] = msg->data[0];
    targetXYold[1] = msg->data[1];
}

void targetXYCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    targetXY[0] = msg->data[0];
    targetXY[1] = msg->data[1];
}

void endEffectorXYCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    endEffectorXY[0] = msg->data[0];
    endEffectorXY[1] = msg->data[1];
}

void targetsCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    for (int i = 0; i < 6; i++) {
        targets[i][0] = msg->data[2*i];
        targets[i][1] = msg->data[2*i + 1];
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gui_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_targetXYold = nh.subscribe("PreviousTargets", 10, targetXYoldCallback);
    ros::Subscriber sub_targetXY = nh.subscribe("CurrentTargets", 10, targetXYCallback);
    ros::Subscriber sub_endEffectorXY = nh.subscribe("CurrentEndEffectorPose", 10, endEffectorXYCallback);
    ros::Subscriber sub_targets = nh.subscribe("TrialTargets", 10, targetsCallback);

    memset(gui_data, 0, sizeof(double) * 40);						// xy coordinates that are sent to gui

    double rangex_ = -0.18;
    double rangex = 0.18;
    double rangey_ = -0.18;
    double rangey =0.18;
    double d_r = 0.015;
    double ex_r = 0.015;
    double u_r = 0.005;
    int unit = 5;
    int flag_unitchange = 0;
    int guiMode = 2;
    int guiMode2 = 1;
    int guiMode3 = 1;
    int radius_e = 0.015-0.002;

    ros::Rate loop_rate(10);  // 10 Hz
    while (ros::ok()) {
        gui_data[0] = (double) guiMode;
        gui_data[1] = targetXYold(0);
        gui_data[2] = targetXYold(1);
        gui_data[3] = d_r;
        gui_data[4] = endEffectorXY(0);
        gui_data[5] = endEffectorXY(1);
        gui_data[6] = u_r;
        gui_data[7] = targetXY(0);
        gui_data[8] = targetXY(1);
        gui_data[9] = ex_r;
        gui_data[10] = (double) 0 //stiffness(2, 2);
        gui_data[11] = 7 //(float) (trialNum);
        gui_data[12] = (float) (7);
        gui_data[13] = (double) 0 //stiffness(0, 0);
        gui_data[14] = (double) 0;//beep_flag;
        gui_data[15] = (double) guiMode2;
        gui_data[16] = (double) guiMode3;
        gui_data[17] = (double) flag_unitchange;
        gui_data[18] = (double) (unit + 1);
        gui_data[19] = targets[0][0];
        gui_data[20] = targets[0][1];
        gui_data[21] = targets[1][0];
        gui_data[22] = targets[1][1];
        gui_data[23] = targets[2][0];
        gui_data[24] = targets[2][1];
        gui_data[25] = targets[3][0];
        gui_data[26] = targets[3][1];
        gui_data[27] = targets[4][0];

        udp_server.Send(gui_data, 40);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
