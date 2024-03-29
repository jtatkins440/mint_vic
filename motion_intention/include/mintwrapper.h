#include <torch/script.h> // One-stop header.
#include <nlohmann/json.hpp>
#include <Eigen/Dense>
#include <chrono>
#include "spline.h"

//using json = nlohmann::json;
//using namespace torch::indexing;

#include "circle.h"
#include "circleutils.h"
#include "circlefit.h"

// TODO
// make a struct for outputting the line, circle, and input/output info, make that struct a message and publish it
struct MIntNetIO{
	int input_seq_length;
	std::vector<double> input_times;
	std::vector<std::vector<double>> input_vel; // sequence of input velocity vectors
	std::vector<std::vector<double>> input_acc; // sequence of output acceleration vectors

	int output_seq_length;
	std::vector<double> output_times; // time index for each output_dp[i] vector
	std::vector<std::vector<double>> output_dpos; // sequence of output differential position vectors

	MIntNetIO(){};
	// constructor
	MIntNetIO(int input_seq_length_,
	std::vector<double> input_times_,
	std::vector<std::vector<double>> input_vel_,
	std::vector<std::vector<double>> input_acc_,
	int output_seq_length_,
	std::vector<double> output_times_,
	std::vector<std::vector<double>> output_dpos_){
		input_seq_length = input_seq_length_;
		input_times = input_times_;
		input_vel = input_vel_;
		input_acc = input_acc_;
		output_seq_length = output_seq_length_;
		output_times = output_times_;
		output_dpos = output_dpos_;
	};
};

struct LineFitIO{
	int input_seq_length;
	std::vector<double> input_times;
	std::vector<std::vector<double>> input_pos; // sequence of input position vectors

	int output_dim;
	std::vector<double> output_intercept;
	std::vector<double> output_slope;
	
	LineFitIO(){};
	// constructor
	LineFitIO(int input_seq_length_,
	std::vector<double> input_times_,
	std::vector<std::vector<double>> input_pos_,
	int output_dim_,
	std::vector<double> output_intercept_,
	std::vector<double> output_slope_){
		input_seq_length = input_seq_length_;
		input_times = input_times_;
		input_pos = input_pos_;
		output_dim = output_dim_;
		output_intercept = output_intercept_;
		output_slope = output_slope_;
	}
};

struct CircleFitIO{
	int input_seq_length;
	std::vector<double> input_times;
	std::vector<std::vector<double>> input_pos; // sequence of input position vectors

	int output_dim;
	std::vector<double> circle_center; // [circle.a, circle.b] for (x,y) center of circle
	double circle_radius;

	CircleFitIO(){};
	// constructor
	CircleFitIO(int input_seq_length_,
	std::vector<double> input_times_,
	std::vector<std::vector<double>> input_pos_,
	int output_dim_,
	std::vector<double> circle_center_,
	double circle_radius_){
		input_seq_length = input_seq_length_;
		input_times = input_times_;
		input_pos = input_pos_;
		output_dim = output_dim_;
		circle_center = circle_center_;
		circle_radius = circle_radius_;
	};
};

class MIntSpline{
	public:
	MIntSpline(){};

	MIntSpline(std::vector<double> spline_time, int spline_dim, double lead_time) {
		// put in the current state's time and append the times for the prediction vector
		for (int i = 0; i < spline_time.size(); i++) {
			spline_time_vec.push_back(spline_time[i]);
		}
		
		// one spline for each dim of the prediction
		_spline_dim = spline_dim;
		for (int i = 0; i < spline_dim; i++) {
			tk::spline s;
			spline_vec.push_back(s);
		}
		equilibrium_lead_time = lead_time;
		max_sample_time = spline_time_vec[spline_time_vec.size() - 1]; // don't allow sampling further than the end of the prediction window
	};

	void updateSpline(Eigen::ArrayXf current_state, Eigen::ArrayXXf pred_pos);
	Eigen::ArrayXf sampleSpline(double sample_time);
	Eigen::ArrayXf sampleEquilibriumPoint();
	double equilibrium_lead_time;
	

	private:
	int _spline_dim;
	std::vector<tk::spline> spline_vec;
	std::vector<double> spline_time_vec;
	std::chrono::time_point<std::chrono::steady_clock> spline_timer_start;
	double max_sample_time;
};

void MIntSpline::updateSpline(Eigen::ArrayXf current_state, Eigen::ArrayXXf pred_pos) {

	// shifts relative positions to global 
	std::vector<tk::spline> spline_vec_temp;
	for (int i = 0; i < _spline_dim; i++) {
		for (int j = 0; j < pred_pos.cols(); j++) {
			pred_pos(i, j) = pred_pos(i, j) + current_state(i);
		}
		std::vector<double> spline_points;

		for (int j = 0; j < pred_pos.cols(); j++) {
			spline_points.push_back((double) pred_pos(i, j));
		}

		double end_vel_est = (pred_pos(i, pred_pos.cols()-1) - pred_pos(i, pred_pos.cols()-2)) / (spline_time_vec[spline_time_vec.size()-1] - spline_time_vec[spline_time_vec.size()-2]);
		tk::spline s;
		s.set_boundary(tk::spline::first_deriv, (double) current_state(i + _spline_dim), 
			tk::spline::first_deriv, end_vel_est);

		s.set_points(spline_time_vec, spline_points); // one spline for each dim of the prediction
		spline_vec_temp.push_back(s);
	}
	spline_vec.swap(spline_vec_temp);
	spline_timer_start = std::chrono::steady_clock::now(); // time point for when the spline was last updated
	return;
};

Eigen::ArrayXf MIntSpline::sampleSpline(double sample_time){
	if (max_sample_time < sample_time)
	{
		sample_time = max_sample_time;
	}

	Eigen::ArrayXf sampled_point(_spline_dim, 1);

	for (int i = 0; i < _spline_dim; i++) {sampled_point(i, 1) = (float) spline_vec[i](sample_time);}
	return sampled_point;
};

Eigen::ArrayXf MIntSpline::sampleEquilibriumPoint(){
	Eigen::ArrayXf sampled_point(_spline_dim, 1);
	auto end = std::chrono::steady_clock::now();
	std::chrono::duration<double> diff = end - spline_timer_start;
	double spline_eq_time = diff.count() + equilibrium_lead_time;
	if (max_sample_time < spline_eq_time)
	{
		spline_eq_time = max_sample_time;
	}
	for (int i = 0; i < _spline_dim; i++) {
		sampled_point(i) = (float) spline_vec[i](spline_eq_time);
	}

	return sampled_point;
};

class MIntWrapper{
	public:
	MIntWrapper(){};
	
	MIntWrapper(std::string model_path, std::string json_path) : mint_path(model_path), param_path(json_path) {
		// guard this in a try/catch block!
		
		// read in json and initalize parameters
		std::ifstream f(param_path);
		params = nlohmann::json::parse(f);
		nlohmann::json data_helper = params["helper_params"];
		nlohmann::json data_model = params["mdl_params"];
		input_chn_size = int(data_model["input_size"]);
		output_chn_size = int(data_model["output_size"]);
		input_seq_length = int(data_helper["input_sequence_length"]);
		output_seq_length = int(data_model["M"]) * int(data_model["G"]);
		eq_lead_time = (double) data_helper["lead_time"];
		model_dim = int(data_helper["dim"]);
		// unpack scaling values
		std::vector<float> input_scalers;
		for (int i = 0; i < input_chn_size; i++) {
			input_scalers.push_back(data_helper["input_scaling_list"][i]);
		}
		std::cout << input_scalers << std::endl;

		std::vector<float> output_scalers;
		for (int i = 0; i < output_chn_size; i++) {
			output_scalers.push_back(data_helper["output_scaling_list"][i]);
		}
		
		// create scaling arrays
		input_scaling_array = Eigen::ArrayXXf::Ones(input_chn_size, input_seq_length); // should change all of these to Eigen::Array<float, chn_size, seq_length> and similar for better optimization. No need for allocating during runtime but maybe not big issue here.
		output_scaling_array = Eigen::ArrayXXf::Ones(output_chn_size, output_seq_length);
		for (int i = 0; i < input_chn_size; i++) {
			for (int j = 0; j < input_seq_length; j++) {
				input_scaling_array(i, j) = input_scalers[i];
			}
		}

		for (int i = 0; i < output_chn_size; i++) {
		  for (int j = 0; j < output_seq_length; j++) {
		    output_scaling_array(i, j) = output_scalers[i];
		  }
		}
		
		// load mintnet itself
		try {
			mint_module = torch::jit::load(mint_path);
		}
		catch (const c10::Error& e) {
			std::cerr << "error loading the model\n";
			return ;
		}
		
		input_a = Eigen::ArrayXXf::Ones(input_chn_size, input_seq_length);
		output_a = Eigen::ArrayXXf::Ones(output_chn_size, output_seq_length);

		// initialize I/O tensors
		input_t = torch::randn({1, input_chn_size, input_seq_length});
		output_t = torch::randn({1, output_chn_size, output_seq_length});
		
		double dt = (double) data_helper["dt"];
		input_time_vec.clear();
		for (int i = 0; i < input_seq_length; i++) {
			double time_point = -((double)(input_seq_length - i - 1)) * dt;
			input_time_vec.push_back(time_point);
		}

		output_time_vec.clear();
		for (int i = 0; i < output_seq_length; i++) {
			output_time_vec.push_back(data_helper["output_times"][i]);
		}

		mintspline = MIntSpline(output_time_vec, output_chn_size, eq_lead_time);

		io_struct = MIntNetIO();
		// finished
		std::cout << "MIntWrapper Initalized!" << std::endl;
	}
	double eq_lead_time;

	Eigen::ArrayXXf forward(std::deque<Eigen::ArrayXf> input);
	Eigen::ArrayXXf forward(Eigen::ArrayXXf input);
	
	// main two functions that need to be implimented in every MIntWrapper variation are fit() and getEquilibriumPoint()
	void fit(Eigen::ArrayXf current_state, std::deque<Eigen::ArrayXf> input);
	void fit(Eigen::ArrayXf current_state, Eigen::ArrayXXf input);
	MIntNetIO io_struct;
	MIntNetIO getIOStruct(); // getter function for the inputs and outputs
	Eigen::ArrayXf getEquilibriumPoint();
	int input_chn_size;
	int output_chn_size;
	int input_seq_length;
	int output_seq_length;
	int model_dim;
	std::vector<double> output_time_vec;
	std::vector<double> input_time_vec;
	Eigen::ArrayXXf getInputArray();
	Eigen::ArrayXXf getOutputArray();

	private:
	std::string mint_path;
	std::string param_path;
	nlohmann::json params;
	Eigen::ArrayXXf input_scaling_array;
	Eigen::ArrayXXf output_scaling_array;
	torch::jit::script::Module mint_module;
	at::Tensor input_t;
	at::Tensor output_t;
    torch::jit::IValue output_v;
	std::vector<torch::jit::IValue> input_v;
    Eigen::ArrayXXf input_a;
	Eigen::ArrayXXf output_a;
    Eigen::ArrayXXf forward_(Eigen::ArrayXXf input);
	
	MIntSpline mintspline;
};

Eigen::ArrayXXf MIntWrapper::forward_(Eigen::ArrayXXf input)
{
	// scale the input array
	input_a = input;
	input = input * input_scaling_array;
	
	// convert input Eigen array to Tensor
	input_t = input_t.index_put_({torch::indexing::Slice(torch::indexing::None, input_chn_size, input_seq_length)}, torch::from_blob(input.data(), {input_chn_size, input_seq_length}).clone());
  
	// clear out IValue vector and fill with new input
	input_v.clear();
	input_v.push_back(input_t);

	// predict new unscaled IValue
	output_v = mint_module.forward(input_v);

	// IValue to Tensor
	output_t = output_v.toTensor().index({torch::indexing::Slice(torch::indexing::None, output_chn_size, output_seq_length)}); 

	// Tensor to flat float vector
	std::vector<float> v(output_t.data_ptr<float>(), output_t.data_ptr<float>() + output_t.numel()); 

	// float vector to Eigen array
	int k = 0;
	for (int i = 0; i < output_chn_size; i++) {
		for (int j = 0; j < output_seq_length; j++) {
			output_a(i, j) = v[k]; // fills jth element of vector sequence with ith channel from vector 
			k++;
		}
	}

	// fix Eigen array scaling
	output_a = output_a * output_scaling_array;

	return output_a;
};

// wrapper method for predictions, handles all conversions for ease of online use.
Eigen::ArrayXXf MIntWrapper::forward(std::deque<Eigen::ArrayXf> input)
{
	// converts deque of float 1-d arrays to normal float array
	for (int i = 0; i < input_chn_size; i++) {
		for (int j = 0; j < input_seq_length; j++) {
		input_a(i, j) = input[j](i);
		}
	}
	output_a = forward_(input_a);
	return output_a;
};

// wrapper method for predictions, handles all conversions for ease of online use. Overloaded for variety of inputs
Eigen::ArrayXXf MIntWrapper::forward(Eigen::ArrayXXf input)
{
	output_a = forward_(input);
	return output_a;
};

void MIntWrapper::fit(Eigen::ArrayXf current_state, std::deque<Eigen::ArrayXf> input)
{
	Eigen::ArrayXXf pred_pos = forward(input);
	mintspline.updateSpline(current_state, pred_pos);
};

void MIntWrapper::fit(Eigen::ArrayXf current_state, Eigen::ArrayXXf input)
{
	Eigen::ArrayXXf pred_pos = forward(input);
	mintspline.updateSpline(current_state, pred_pos);
};

Eigen::ArrayXf MIntWrapper::getEquilibriumPoint()
{
	return mintspline.sampleEquilibriumPoint();
};

Eigen::ArrayXXf MIntWrapper::getInputArray(){
	Eigen::ArrayXXf returned_input = input_a;
	return returned_input;
};

// call after forward
Eigen::ArrayXXf MIntWrapper::getOutputArray(){
	Eigen::ArrayXXf returned_output = output_a;
	return returned_output;
};

/*
io_struct = MIntNetIO(int input_seq_length_,
	std::vector<double> input_times_,
	std::vector<std::vector<double>> input_vel_,
	std::vector<std::vector<double>> input_acc_,
	int output_seq_length_,
	std::vector<double> output_times_,
	std::vector<std::vector<double>> output_dpos_);
*/
MIntNetIO MIntWrapper::getIOStruct(){
	std::vector<std::vector<double>> input_vel;
	std::vector<std::vector<double>> input_acc;
	std::vector<std::vector<double>> output_dpos;

	Eigen::ArrayXXf inputs = getInputArray();
	
	std::vector<double> vel; // this is velocity for a single channel across all cols now! Was backwards but this is easier to save
	std::vector<double> acc;
	for (int row = 0; row < model_dim; row++){
		for (int col = 0; col < inputs.cols(); col++){
			vel.push_back(inputs(row, col));
			acc.push_back(inputs(row + model_dim, col));
		}
		input_vel.push_back(vel);
		input_acc.push_back(acc);
		vel.clear(); 
		acc.clear();
	}

	std::vector<double> dpos;
	Eigen::ArrayXXf outputs = getOutputArray();
	for (int row = 0; row < model_dim; row++){
		for (int col = 0; col < outputs.cols(); col++){
			dpos.push_back(outputs(row, col));
		}
		output_dpos.push_back(dpos);
		dpos.clear();
	}

	io_struct = MIntNetIO(input_seq_length,
						input_time_vec,
						input_vel,
						input_acc,
						output_seq_length,
						output_time_vec,
						output_dpos);
	return io_struct;
};

// LineFitWrapper expects to be passed the current state as a [position; velocity] vector and the input as a 2x125 array of 
class LineFitWrapper{
	public:

	nlohmann::json params;
	std::string param_path;
	int input_chn_size;
	int output_chn_size;
	int input_seq_length;
	int used_seq_length;
	int output_seq_length;
	int model_order;
	float equilibrium_lead_time;
	float dt;
	int trim_seq_length;
	float time_scalar;
	std::chrono::time_point<std::chrono::steady_clock> timer_start;
	Eigen::MatrixXf A;
	Eigen::MatrixXf b_coeffs;
	LineFitIO io_struct;
	Eigen::MatrixXf Y_copy;

	LineFitWrapper(){};

	LineFitWrapper(std::string json_path) : param_path(json_path) {
		std::ifstream f(param_path);
		params = nlohmann::json::parse(f);
		nlohmann::json data_helper = params["helper_params"];
		nlohmann::json data_model = params["mdl_params"];
		input_chn_size = int(data_model["input_size"]);
		output_chn_size = int(data_model["output_size"]);
		input_seq_length = int(data_helper["input_sequence_length"]);
		output_seq_length = int(data_model["M"]) * int(data_model["G"]);
		equilibrium_lead_time = (float) data_helper["lead_time"];
		time_scalar = (float) data_helper["linear_lead_time_scalar"];
		trim_seq_length = (int) data_helper["input_sequence_trim_length"];
		dt = (float) data_helper["dt"];
		model_order = int(data_helper["dim"]);

		used_seq_length = input_seq_length - trim_seq_length; // probably should check if non-positive

		// generate observation 'X' array as a 125x2 array where column 0 is just 1's and column 1 is time corresponding to each input. Should be static.
		//Eigen::MatrixXf A(input_seq_length, output_chn_size);
		A = Eigen::MatrixXf::Zero(used_seq_length, output_chn_size);
		std::cout << "LineFitWrapper Constructor: " << "A.rows(): " << A.rows() << ", A.cols(): " << A.cols() << std::endl;
		for (int row = 0; row < used_seq_length; row++){
			A(row, 0) = 1.0;
			A(row, 1) = -((float)(used_seq_length - row - 1)) * dt;
		}
		//std::cout << "A.rows(): " << A.rows() << ", A.cols(): " << A.cols() << std::endl;
		
		//A_QR = A.fullPivHouseholderQR(); // QR decomp of A. to solve matrix eq, just do A_QR.solve(Y) where Y are the 'inputs' or the positions for each dimension.
		
		// 
		Y_copy = Eigen::MatrixXf::Zero(model_order, used_seq_length);

		b_coeffs = Eigen::MatrixXf::Zero(model_order, output_chn_size);
		b_coeffs << 0.0, 0.0, 
					0.0, 0.0; // rows should be [b_0; b_1], cols should be dims [x, y];
		
		io_struct = LineFitIO();

		timer_start = std::chrono::steady_clock::now();
	};

	//void fit(Eigen::ArrayXf current_state, Eigen::ArrayXXf input);
	//Eigen::ArrayXf getEquilibriumPoint();

	/*
	LineFitIO(int input_seq_length_,
	std::vector<double> input_times_,
	std::vector<std::vector<double>> input_pos_,
	int output_dim_,
	std::vector<std::vector<double>> output_intercept_,
	std::vector<std::vector<double>> output_slope_)
	std::vector<std::vector<double>> output_slope_
	*/

	/*
	b_coeffs = Eigen::MatrixXf::Zero(model_order, output_chn_size);
	b_coeffs << 0.0, 0.0, 
				0.0, 0.0; // rows should be [b_0; b_1], cols should be dims [x, y];
	*/
	LineFitIO getIOStruct(){
		std::vector<double> output_intercept;
		std::vector<double> output_slope;

		for (int dim = 0; dim < model_order; dim++){
			output_intercept.push_back(b_coeffs(0, dim));
			output_slope.push_back(b_coeffs(1, dim));
		}
		
		std::vector<double> input_times;
		for (int row = 0; row < used_seq_length; row++){
			input_times.push_back((double) A(row, 1)); //-((float)(used_seq_length - row - 1)) * dt;
		}

		std::vector<std::vector<double>> input_pos;
		std::vector<double> pos;
		for (int row = 0; row < model_order; row++){
			for (int col = 0; col < Y_copy.cols(); col++){
				pos.push_back(Y_copy(row, col));
			}
			input_pos.push_back(pos);
			pos.clear(); 
		}
		
		io_struct = LineFitIO(used_seq_length,
						input_times,
						input_pos,
						output_chn_size,
						output_intercept,
						output_slope);
		return io_struct;
	};

	/* // old
	LineFitIO getIOStruct(){
		std::vector<double> output_intercept;
		std::vector<double> output_slope;

		for (int dim = 0; dim < model_order; dim++){
			output_intercept.push_back(b_coeffs(0, dim));
			output_slope.push_back(b_coeffs(1, dim));
		}
		
		std::vector<double> input_times;
		for (int row = 0; row < used_seq_length; row++){
			input_times.push_back((double) A(row, 1)); //-((float)(used_seq_length - row - 1)) * dt;
		}

		std::vector<std::vector<double>> input_pos;
		std::vector<double> pos;
		for (int col = 0; col < Y_copy.cols(); col++){
			for (int row = 0; row < model_order; row++){
				pos.push_back(Y_copy(row, col));
			}
		input_pos.push_back(pos);
		pos.clear(); 
		}
		
		io_struct = LineFitIO(used_seq_length,
						input_times,
						input_pos,
						output_chn_size,
						output_intercept,
						output_slope);
		return io_struct;
	};
	*/

	void fit(Eigen::ArrayXf current_state, Eigen::ArrayXXf input){
		// current state is ignored, only here because of compatibility with ROS wrapper
		// input is assumed to be 2x125 of [p_x, p_y] values.
		Y_copy = input.bottomRightCorner(model_order, used_seq_length);

		Eigen::MatrixXf Y_full = input.matrix().bottomRightCorner(input.rows(), used_seq_length).transpose();

		
		// rebuild A (shouldn't need to do this outside of constructor, weird behavior)
		//Eigen::MatrixXf A(input_seq_length, output_chn_size);
		//std::cout << "LineFitWrapper fit: " << "Before MatrixXf::Zero, input_seq_length: " << input_seq_length << ", output_chn_size: " << output_chn_size << std::endl;
		//A = Eigen::MatrixXf::Zero(input_seq_length, output_chn_size);
		//std::cout << "LineFitWrapper fit: " << "A.rows(): " << A.rows() << ", A.cols(): " << A.cols() << std::endl;
		//for (int row = 0; row < input_seq_length; row++){
		//	A(row, 0) = 1.0;
		//	A(row, 1) = ((float)(input_seq_length - row - 1)) * dt;
		//}
		

		// find b coeffs for each dim.
		for (int dim = 0; dim < output_chn_size; dim++){
			//Eigen::MatrixXXf b_col_temp = A_QR.solve(Y_full(all, dim));
			//Eigen::MatrixXf Y = Y_full(Eigen::all, dim);
			Eigen::VectorXf Y = Y_full(Eigen::all, dim);
			//std::cout << "Y.rows(): " << Y.rows() << ", Y.cols(): " << Y.cols() << std::endl;
			//std::cout << "A.rows(): " << A.rows() << ", A.cols(): " << A.cols() << std::endl;
			//std::cout << "vals are: " << A.fullPivHouseholderQr().solve(Y) << std::endl;
			b_coeffs(Eigen::all, dim) = A.fullPivHouseholderQr().solve(Y);
		}
		timer_start = std::chrono::steady_clock::now();
	};

	Eigen::ArrayXf getEquilibriumPoint(){
		auto end = std::chrono::steady_clock::now();
		std::chrono::duration<double> diff = end - timer_start;
		double spline_eq_time = time_scalar * (diff.count() + equilibrium_lead_time); // have to flip the direction to make it work as desired

		Eigen::ArrayXf eq_point(output_chn_size);
		eq_point << b_coeffs(0,0) + b_coeffs(1, 0) * spline_eq_time, b_coeffs(0,1) + b_coeffs(1, 1) * spline_eq_time;

		return eq_point;
	}; 

	private:
	

	
};


// CircleFitWrapper expects to be passed the current state as a [position; velocity] vector and the input as a 2x125 array of 
class CircleFitWrapper{
	public:
	CircleFitWrapper(){};

	CircleFitWrapper(std::string json_path) : param_path(json_path) {
		std::ifstream f(param_path);
		params = nlohmann::json::parse(f);
		nlohmann::json data_helper = params["helper_params"];
		nlohmann::json data_model = params["mdl_params"];
		input_chn_size = int(data_model["input_size"]);
		output_chn_size = int(data_model["output_size"]);
		input_seq_length = int(data_helper["input_sequence_length"]);
		output_seq_length = int(data_model["M"]) * int(data_model["G"]);
		eq_lead_time = (float) data_helper["lead_time"];
		time_scalar = (float) data_helper["circle_lead_time_scalar"];
		dt = (float) data_helper["dt"];

		trim_seq_length = (int) data_helper["input_sequence_trim_length"];
		used_seq_length = input_seq_length - trim_seq_length;

		// generate observation 'X' array as a 125x2 array where column 0 is just 1's and column 1 is time corresponding to each input. Should be static.

		// TODO: add circle stuff too
		//Eigen::MatrixXf x_eq(output_chn_size);
		x_eq = Eigen::MatrixXf::Zero(output_chn_size, 1);
		Circle circle;
		io_struct = CircleFitIO();

		
		for (int row = 0; row < used_seq_length; row++){
			input_times.push_back(-((double)(used_seq_length - row - 1)) * dt);
		}

		timer_start = std::chrono::steady_clock::now();
	};

	//void fit(Eigen::ArrayXf current_state, Eigen::ArrayXXf input);
	//Eigen::ArrayXf getEquilibriumPoint();
	Circle circle;
	CircleFitIO io_struct;
	std::vector<double> x_vec;
	std::vector<double> y_vec;
	std::vector<double> input_times;
	/*
	CircleFitIO(int input_seq_length_,
	std::vector<double> input_times_,
	std::vector<std::vector<double>> input_pos_,
	std::vector<double> circle_center_,
	double circle_radius_)
	*/

	CircleFitIO getIOStruct(){
		std::vector<std::vector<double>> input_pos;

		input_pos.push_back(x_vec);
		input_pos.push_back(y_vec);

		std::vector<double> circle_center = {circle.a, circle.b};
		io_struct = CircleFitIO(used_seq_length,
							input_times,
							input_pos,
							output_chn_size,
							circle_center,
							(double) circle.r);
		return io_struct;
	};

	/*
	CircleFitIO getIOStruct(){
		std::vector<std::vector<double>> input_pos;
		std::vector<double> pos;
		for (int col = 0; col < used_seq_length; col++){
			pos.push_back(x_vec[col]);
			pos.push_back(y_vec[col]);

			input_pos.push_back(pos);
			pos.clear(); 
		}

		std::vector<double> circle_center = {circle.a, circle.b};
		io_struct = CircleFitIO(used_seq_length,
							input_times,
							input_pos,
							output_chn_size,
							circle_center,
							(double) circle.r);
		return io_struct;
	};
	*/

	// UPDATE
	void fit(Eigen::ArrayXf current_state, Eigen::ArrayXXf input){
		// current state is ignored, only here because of compatibility with ROS wrapper
		// input is assumed to be 2x125 of [p_x, p_y] values.

		int size = used_seq_length; //input.cols();
		double x_array[size];
		double y_array[size];
		x_vec.clear();
		y_vec.clear();
		for (int i = 0; i < size; i++){
			x_array[i] = input(0, i + trim_seq_length); // x_pos
			y_array[i] = input(1, i + trim_seq_length); // y_pos

			x_vec.push_back(x_array[i]);
			y_vec.push_back(y_array[i]);
		}

		CircleData Datafitcircle(size, x_array, y_array);
		circle = CircleFitByPratt(Datafitcircle);

		Eigen::ArrayXf leading_state = current_state;
		leading_state(0) = leading_state(0) + time_scalar * eq_lead_time * leading_state(2);
		leading_state(1) = leading_state(1) + time_scalar * eq_lead_time * leading_state(3);
		x_eq(0) = circle.a + circle.r*(leading_state(0)-circle.a)/(sqrt(pow(leading_state(0)-circle.a,2)+pow(leading_state(1)-circle.b,2)));
		x_eq(1) = circle.b + (x_eq(0)-circle.a)*(leading_state(1)-circle.b)/(leading_state(0)-circle.a);
		
		//x_eq(0) = circle.a + circle.r*(current_state(0)-circle.a)/(sqrt(pow(current_state(0)-circle.a,2)+pow(current_state(1)-circle.b,2)));
		//x_eq(1) = circle.b + (x_eq(0)-circle.a)*(current_state(1)-circle.b)/(current_state(0)-circle.a);
		//x_eq(0) = projected(0);
		//x_eq(1) = projected(1);
		//angleproj = atan2(projected(1)-xcurrent(1),projected(0)-xcurrent(0));

		timer_start = std::chrono::steady_clock::now(); // timing not used in OG implimentation, just return same ol' point
	}

	// UPDATE
	Eigen::ArrayXf getEquilibriumPoint(){
		return x_eq;
	}; 
	
	private:
	std::string param_path;
	nlohmann::json params;
	int input_chn_size;
	int output_chn_size;
	int input_seq_length;
	int output_seq_length;
	int trim_seq_length;
	int used_seq_length;
	int model_order;
	float eq_lead_time;
	float dt;
	float time_scalar;
	std::chrono::time_point<std::chrono::steady_clock> timer_start;
	//Eigen::FullPivHouseholderQR:::FullPivHouseholderQR<Eigen::MatrixXf> A_QR;
	Eigen::MatrixXf A;
	Eigen::MatrixXf x_eq;
	


	
};
