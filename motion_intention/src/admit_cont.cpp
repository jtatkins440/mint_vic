#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "motion_intention/HistoryStamped.h"
#include "motion_intention/SetInt.h"
#include "motion_intention/FullFitIO.h"
#include <sstream>
#include "mintwrapper.h"
#include <iostream>
#include <fstream>
#include <memory>
#include <deque>
#include <chrono>
#include <mutex>
#include <thread>
#include <cmath>

// replace hardcoded dims with these
#define STATE_DIM 3
#define FULL_STATE_DIM 6

// admittance controller with 3-d postion (6-d total state) as assumed domain
class PositionAdmittanceController{
	public:
		PositionAdmittanceController(){
			inertia_matrix = Eigen::Matrix<double, 3, 3>::Identity();
			inv_inertia_matrix = Eigen::Matrix<double, 3, 3>::Identity();
			damping_matrix = Eigen::Matrix<double, 3, 3>::Identity();
			stiffness_matrix = Eigen::Matrix<double, 3, 3>::Zero();
			A_matrix = Eigen::Matrix<double, 6, 6>::Zero();
			B_matrix = Eigen::Matrix<double, 6, 6>::Zero();
			F_matrix = Eigen::Matrix<double, 6, 3>::Zero();
			state_vector = Eigen::Matrix<double, 6, 1>::Zero();
			dstate_vector = Eigen::Matrix<double, 6, 1>::Zero();
			temp_state_vector = Eigen::Matrix<double, 6, 1>::Zero();
			temp_dstate_vector = Eigen::Matrix<double, 6, 1>::Zero();
			//disturbance_vector = Eigen::Matrix<double, 6, 1>::Zero();
		}

	Eigen::Matrix<double, 3, 3> inertia_matrix;
	Eigen::Matrix<double, 3, 3> inv_inertia_matrix; // useful to precompute for later
	Eigen::Matrix<double, 3, 3> damping_matrix; 
	Eigen::Matrix<double, 3, 3> stiffness_matrix;

	Eigen::Matrix<double, 6, 6> A_matrix;
	Eigen::Matrix<double, 6, 6> B_matrix; 
	Eigen::Matrix<double, 6, 3> F_matrix;

	Eigen::Matrix<double, 6, 1> state_vector; // [pos; vel]_i state vector
	Eigen::Matrix<double, 6, 1> dstate_vector; // [vel; acc]_i, derivative of state vector

	Eigen::Matrix<double, 6, 1> temp_state_vector; // [pos; vel]_i state vector
	Eigen::Matrix<double, 6, 1> temp_dstate_vector; // [vel; acc]_i, derivative of state vector

	//Eigen::Matrix<double, 6, 1> disturbance_vector; // disturbance forces, F_matrix * disturbance_vector = state change due to disturbances

	bool setDynamicsMatrices(Eigen::Matrix<double, 3, 3> new_inertia_matrix, Eigen::Matrix<double, 3, 3> new_damping_matrix, Eigen::Matrix<double, 3, 3> new_stiffness_matrix); // updates A, B, and F given new inertia, damping, and stiffness matrices. Should be called after I, B, K are updated!

	Eigen::Matrix<double, 6, 1> computeDynamicsStep(Eigen::Matrix<double, 6, 1> eq_state, Eigen::Matrix<double, 3, 1> force, double dt);
	bool doDynamicsStep(Eigen::Matrix<double, 6, 1> eq_state, Eigen::Matrix<double, 3, 1> force, double dt);
};

bool PositionAdmittanceController::setDynamicsMatrices(Eigen::Matrix<double, 3, 3> new_inertia_matrix, Eigen::Matrix<double, 3, 3> new_damping_matrix, Eigen::Matrix<double, 3, 3> new_stiffness_matrix){
	// check if inertia_matrix is invertible first, if not return false
	// TODO

	// save some local copies of the matrices
	inertia_matrix = new_inertia_matrix;
	inv_inertia_matrix = new_inertia_matrix.inverse();

	damping_matrix = new_damping_matrix;
	stiffness_matrix = new_stiffness_matrix;

	A_matrix.block<3,3>(0,0) = Eigen::Matrix<double, 3, 3>::Zero(); // block of size <p, q> starting at index (i, j)
	A_matrix.block<3,3>(0,3) = Eigen::Matrix<double, 3, 3>::Identity();
	A_matrix.block<3,3>(3,0) = -inv_inertia_matrix * stiffness_matrix;
	A_matrix.block<3,3>(3,3) = -inv_inertia_matrix * damping_matrix;

	B_matrix.block<3,3>(0,0) = Eigen::Matrix<double, 3, 3>::Zero(); // block of size <p, q> starting at index (i, j)
	B_matrix.block<3,3>(0,3) = Eigen::Matrix<double, 3, 3>::Zero();
	B_matrix.block<3,3>(3,0) = inv_inertia_matrix * stiffness_matrix;
	B_matrix.block<3,3>(3,3) = inv_inertia_matrix * damping_matrix;

	F_matrix.block<3,3>(0,0) = Eigen::Matrix<double, 3, 3>::Zero(); // block of size <p, q> starting at index (i, j)
	F_matrix.block<3,3>(3,0) = inv_inertia_matrix;

	//disturbance_vector = Eigen::Matrix<double,6,1>::Zero();
	return true;
}

Eigen::Matrix<double, 6, 1> PositionAdmittanceController::computeDynamicsStep(Eigen::Matrix<double, 6, 1> eq_state, Eigen::Matrix<double, 3, 1> force, double dt){
	//disturbance_vector.block<3,1>(3,0) = force;
	temp_dstate_vector = A_matrix * state_vector + B_matrix * eq_state + F_matrix * force;
	return state_vector + dt * temp_dstate_vector;
}

// computes the next state given current state, equilibrium state, and measured forces AND assumes the system moves there and updates internal variables.
bool PositionAdmittanceController::doDynamicsStep(Eigen::Matrix<double, 6, 1> eq_state, Eigen::Matrix<double, 3, 1> force, double dt){
	temp_state_vector = computeDynamicsStep(eq_state, force, dt);
	state_vector = temp_state_vector;
	dstate_vector = temp_dstate_vector;
	return true;
}

/*
class AdmitControlNodeWrapper{
	public:

		AdmitControlNodeWrapper(){
			ros::NodeHandle nh("admittance_controller");
			//nh.getParam
			
			//sub = nh.subscribe("/ee_pose", 2, &MIntNodeWrapper::subscriberCallback, this); // assumes it's reading a task_space PoseStamped message
			//sub_hist = nh.subscribe("/ee_history", 2, &MIntNodeWrapper::subscriberHistoryCallback, this); // assumes it's reading a task_space PoseStamped message
			//pub = nh.advertise<geometry_msgs::PoseStamped>("/ee_pose_eq", 2);
			//pub_io = nh.advertise<motion_intention::FullFitIO>("/full_fit_io", 2);
			
			
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
			

			//nh.param<double>("sample_time", sample_time, 0.005);
			//nh.param<int>("inference_rate", inference_rate, 500);
			//nh.param<int>("state_dim", state_dim, 2);

			ROS_INFO("mint: AdmitControlNodeWrapper initalized.");
		};

		ros::NodeHandle nh;

		void waitRemainingLoopTime(std::chrono::time_point<std::chrono::steady_clock> start_time, std::chrono::duration<double> desired_duration);
		void mainLoop();

	private:
		std::chrono::steady_clock::time_point time_start;
};


void AdmitControlNodeWrapper::waitRemainingLoopTime(std::chrono::time_point<std::chrono::steady_clock> start_time, std::chrono::duration<double> desired_duration){
	std::chrono::duration<double> diff = std::chrono::steady_clock::now() - start_time; 
	while (diff.count() < desired_duration.count()){
		diff = std::chrono::steady_clock::now() - start_time; 
	}
};

void AdmitControlNodeWrapper::mainLoop() {
	
	// initalize anything here


	while (ros::ok()){
		// control loop

		// let callbacks run
		ros::spinOnce();

		// wait remaining loop time
		r.sleep();
	}
	return;	
};
*/

Eigen::Matrix<double,3,3> get_z_rot_mat(double angle){
	Eigen::Matrix<double, 3, 3> rot_mat;
	rot_mat << cos(angle), -sin(angle), 0.0,
				sin(angle), cos(angle), 0.0,
				0.0, 0.0, 1.0;
	return rot_mat;
}

void admit_controller_sim(PositionAdmittanceController pos_admit){
	double force_mag = 2.0;
	double force_ang_vel = 2.0 * 3.14;
	double mass_diag = 1.0;
	double damping_diag = 0.5;
	double stiffness_diag = 0.0;
	double dt = 0.005;
	int simulation_step_num = 2000;
	double end_time = dt * (double) simulation_step_num;
	double time = 0.0;

	Eigen::Matrix<double, 3, 3> inertia = mass_diag * Eigen::Matrix<double, 3, 3>::Identity();
	Eigen::Matrix<double, 3, 3> damping = damping_diag * Eigen::Matrix<double, 3, 3>::Identity();
	Eigen::Matrix<double, 3, 3> stiffness = stiffness_diag * Eigen::Matrix<double, 3, 3>::Identity();

	std::vector<Eigen::Matrix<double,6,1>> state_record;
	std::vector<double> time_step;
	std::vector<double> calculation_time;

	Eigen::Matrix<double,3,1> force_init = {force_mag, 0.0, 0.0};
	Eigen::Matrix<double,3,1> force = force_init;

	Eigen::Matrix<double,3,3> rotmat = get_z_rot_mat(time);
	Eigen::Matrix<double,6,1> eq_state = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	pos_admit.setDynamicsMatrices(inertia, damping, stiffness);

	for (int i = 0; i<simulation_step_num; i++){
		rotmat = get_z_rot_mat(force_ang_vel * time);
		force = rotmat * force_init;

		std::chrono::time_point<std::chrono::steady_clock> start_time = std::chrono::steady_clock::now();
		pos_admit.doDynamicsStep(eq_state, force, dt);
		if (simulation_step_num/2 < i){
			stiffness_diag = 1.0;
			stiffness = stiffness_diag * Eigen::Matrix<double, 3, 3>::Identity();
			pos_admit.setDynamicsMatrices(inertia, damping, stiffness);
		}
		std::chrono::duration<double> diff = std::chrono::steady_clock::now() - start_time; 
		time_step.push_back(time);
		calculation_time.push_back( (double) diff.count() );
		state_record.push_back(pos_admit.state_vector);
		
		time = dt * (double) i;
	}

	// save everything to a csv
	std::ofstream outfile;
	outfile.open("admit_sim.csv");
	outfile << "time, calc_time, px, py, pz, vx, vy, vz\n";
	for (int i = 0; i < simulation_step_num; i++){
		outfile << std::to_string(time_step[i]) << "," << std::to_string(calculation_time[i]);
		for (int j = 0; j < 6; j++){
			outfile << "," << state_record[i][j];
		}
		outfile << "\n";
		
	}
	outfile.close();
	return;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "admittance_controller");
	std::cout << "Starting simulation..." << std::endl;
	PositionAdmittanceController pos_admit;
	admit_controller_sim(pos_admit);
	std::cout << "Simulation finished!" << std::endl;
}
