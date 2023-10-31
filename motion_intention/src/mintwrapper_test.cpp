#include "mintwrapper.h"
#include <iostream>
#include <fstream>
#include <memory>
#include <deque>
#include <random>
#include "ros/ros.h"
#include <cmath>

Eigen::ArrayXXf getLineData(Eigen::ArrayXXf time_domain, float x_slope, float x_intercept, float y_slope, float y_intercept, float noise_scale){
    Eigen::ArrayXXf traj(6, 125);

    std::default_random_engine generator;
    std::normal_distribution<float> distribution(0.0,noise_scale);
    for (int col = 0; col < time_domain.cols(); col++){
        float x_noise_val = 0.0 * distribution(generator);
        float y_noise_val = 0.0 * distribution(generator);
        traj(0,col) = x_intercept + x_slope * time_domain(0,col) + x_noise_val;
        traj(1,col) = y_intercept + y_slope * time_domain(0,col) + y_noise_val;
        traj(2,col) = x_slope + x_noise_val;
        traj(3,col) = y_slope + y_noise_val;
        traj(4,col) = 0.0 + x_noise_val;
        traj(5,col) = 0.0 + y_noise_val;
    }

    return traj;
};

Eigen::ArrayXXf getCircleData(Eigen::ArrayXXf time_domain, float radius, float x_center, float y_center, float phase, float noise_scale){
    std::cout << "before traj def" << std::endl;
    Eigen::ArrayXXf traj(6, 125);

    std::default_random_engine generator;
    std::normal_distribution<float> distribution(0.0,noise_scale);
    std::cout << "before traj filling in" << std::endl;
    for (int col = 0; col < time_domain.cols(); col++){
        float x_noise_val = 0.0 * distribution(generator);
        float y_noise_val = 0.0 * distribution(generator);
        float time = time_domain(0,col) + phase;
        
        traj(0,col) = x_center + radius * std::cos(time) + x_noise_val;
        traj(1,col) = y_center + radius * std::sin(time) + y_noise_val;
        traj(2,col) = x_center - radius * std::sin(time) + x_noise_val;
        traj(3,col) = y_center + radius * std::cos(time) + y_noise_val;
        traj(4,col) = x_center - radius * std::cos(time) + x_noise_val;
        traj(5,col) = y_center - radius * std::sin(time) + y_noise_val;
        //traj(2,col) = x_slope + x_noise_val;
        //traj(3,col) = y_slope + y_noise_val;
        //traj(4,col) = 0.0 + x_noise_val;
        //traj(5,col) = 0.0 + y_noise_val;
    }

    return traj;
};


Eigen::ArrayXXf getArchData(){

    return Eigen::ArrayXXf::Zero(6, 125);
};

int main(int argc, char **argv){
    ros::init(argc, argv, "mintwrapper_test");
    ros::NodeHandle nh("mintwrapper_test");

    std::string model_weights_path;
    std::string hyperparam_weights_path;
    double sample_time;
    int state_dim;
    int seq_length;
    int mint_state_dim;

    // get params, should be sent to param server by launch file.
    if (ros::param::get("/mint/model_weights_path", model_weights_path)) {
        std::cout << "Recieved model_weights_path as " << model_weights_path << std::endl;
    }
    else {
        ROS_INFO("!!!No model_weights_path in mintwrapper_test!!!");
    };

    if (ros::param::get("/mint/hparam_json_path", hyperparam_weights_path)) {
        std::cout << "Recieved hyperparam_weights_path as " << hyperparam_weights_path << std::endl;
    }
    else {
        ROS_INFO("!!!No hyperparam_weights_path in mintwrapper_test!!!");
    };

    ros::param::param<double>("/mint/sample_time", sample_time, 0.005);
    ros::param::param<int>("/mint/state_dim", state_dim, 2);
    ros::param::param<int>("/mint/seq_length", seq_length, 125);
    ros::param::param<int>("/mint/mint_state_dim", mint_state_dim, 4);

    MIntWrapper mintnet = MIntWrapper(model_weights_path, hyperparam_weights_path);

    LineFitWrapper mint_line = LineFitWrapper(hyperparam_weights_path);
    CircleFitWrapper mint_circ = CircleFitWrapper(hyperparam_weights_path);

    std::cout << "before time_domain def." << std::endl;

    // generate three test case data
    Eigen::ArrayXXf time_domain(1, 125);
    for (int i = 0; i < time_domain.cols(); i++){
        time_domain(0, 125 - i - 1) = (float) (-i * sample_time);
    }

    float x_slope = 5.0;
    float x_intercept = 0.0;
    float y_slope = 3.0;
    float y_intercept = 0.0;
    float noise_scale = 0.1;

    Eigen::ArrayXXf line_data = getLineData(time_domain, x_slope, x_intercept, y_slope, y_intercept, noise_scale);

    float radius = 0.1;
    float x_center = 0.0;
    float y_center = 0.1;
    float phase = 3.14 * 3 / 2;
    Eigen::ArrayXXf circle_data = getCircleData(time_domain, radius, x_center, y_center, phase, noise_scale);

    // fit the three models to the three test cases
    Eigen::ArrayXf current_state(4,1);
    current_state << 0.0, 0.0, 0.0, 0.0;
    Eigen::ArrayXXf line_pos = line_data(Eigen::seq(0,1), Eigen::all);
    mint_line.fit(current_state, line_pos);
    std::cout << "mint_line.fit has finished." << std::endl;

    Eigen::ArrayXXf circle_pos = circle_data(Eigen::seq(0,1), Eigen::all);
    mint_circ.fit(current_state, circle_pos);
    std::cout << "mint_circ.fit has finished." << std::endl;
    
    std::cout << "fit radius: " << mint_circ.circle.r << ", fit center: (" << mint_circ.circle.a << ", " << mint_circ.circle.b << ")" << std::endl;

    // sample the fitted models on the domain for compairison
    Eigen::ArrayXf val = mint_line.getEquilibriumPoint();
    std::cout << "fit line val: " << val << std::endl;
    
    // export info to jsons or csvs for plotting the results
    
    return 0;
};

