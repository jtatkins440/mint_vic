#include <torch/script.h> // One-stop header.
#include <nlohmann/json.hpp>
#include <Eigen/Dense>
#include <chrono>
#include "spline.h"

using json = nlohmann::json;
using namespace torch::indexing;

#include "circle.h"
#include "circleutils.h"
#include "circlefit.h"

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
		params = json::parse(f);
		json data_helper = params["helper_params"];
		json data_model = params["mdl_params"];
		input_chn_size = int(data_model["input_size"]);
		output_chn_size = int(data_model["output_size"]);
		input_seq_length = int(data_helper["input_sequence_length"]);
		output_seq_length = int(data_model["M"]) * int(data_model["G"]);
		eq_lead_time = (double) data_helper["lead_time"];
		
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
		    output_scaling_array(i, j) = 1.0 / output_scalers[i];
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
		
		std::vector<double> time_vec;
		for (int i = 0; i < output_seq_length; i++) {
			time_vec.push_back(data_helper["output_times"][i]);
		}

		mintspline = MIntSpline(time_vec, output_chn_size, eq_lead_time);

		// finished
		std::cout << "MIntWrapper Initalized!" << std::endl;
	}
	double eq_lead_time;

	Eigen::ArrayXXf forward(std::deque<Eigen::ArrayXf> input);
	Eigen::ArrayXXf forward(Eigen::ArrayXXf input);
	
	// main two functions that need to be implimented in every MIntWrapper variation are fit() and getEquilibriumPoint()
	void fit(Eigen::ArrayXf current_state, std::deque<Eigen::ArrayXf> input);
	void fit(Eigen::ArrayXf current_state, Eigen::ArrayXXf input);
	Eigen::ArrayXf getEquilibriumPoint();
	int input_chn_size;
	int output_chn_size;
	int input_seq_length;
	int output_seq_length;

	private:
	std::string mint_path;
	std::string param_path;
	json params;
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
	input = input * input_scaling_array;
	
	// convert input Eigen array to Tensor
	input_t = input_t.index_put_({Slice(None, input_chn_size, input_seq_length)}, torch::from_blob(input.data(), {input_chn_size, input_seq_length}).clone());
  
	// clear out IValue vector and fill with new input
	input_v.clear();
	input_v.push_back(input_t);

	// predict new unscaled IValue
	output_v = mint_module.forward(input_v);

	// IValue to Tensor
	output_t = output_v.toTensor().index({Slice(None, output_chn_size, output_seq_length)}); 

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

// LineFitWrapper expects to be passed the current state as a [position; velocity] vector and the input as a 2x125 array of 
class LineFitWrapper{
	public:

	json params;
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
	std::chrono::time_point<std::chrono::steady_clock> timer_start;
	Eigen::MatrixXf A;
	Eigen::MatrixXf b_coeffs;

	LineFitWrapper(){};

	LineFitWrapper(std::string json_path) : param_path(json_path) {
		std::ifstream f(param_path);
		params = json::parse(f);
		json data_helper = params["helper_params"];
		json data_model = params["mdl_params"];
		input_chn_size = int(data_model["input_size"]);
		output_chn_size = int(data_model["output_size"]);
		input_seq_length = int(data_helper["input_sequence_length"]);
		output_seq_length = int(data_model["M"]) * int(data_model["G"]);
		equilibrium_lead_time = (float) data_helper["lead_time"];

		trim_seq_length = (int) data_helper["input_sequence_trim_length"];
		dt = (float) data_helper["dt"];

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
		model_order = 2; // hardcoding is bad! stop it.
		b_coeffs = Eigen::MatrixXf::Zero(model_order, output_chn_size);
		b_coeffs << 0.0, 0.0, 
					0.0, 0.0; // rows should be [b_0; b_1], cols should be dims [x, y];
		
		timer_start = std::chrono::steady_clock::now();
	};

	//void fit(Eigen::ArrayXf current_state, Eigen::ArrayXXf input);
	//Eigen::ArrayXf getEquilibriumPoint();

	void fit(Eigen::ArrayXf current_state, Eigen::ArrayXXf input){
		// current state is ignored, only here because of compatibility with ROS wrapper
		// input is assumed to be 2x125 of [p_x, p_y] values.
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
		double spline_eq_time = (diff.count() + equilibrium_lead_time); // have to flip the direction to make it work as desired

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
		params = json::parse(f);
		json data_helper = params["helper_params"];
		json data_model = params["mdl_params"];
		input_chn_size = int(data_model["input_size"]);
		output_chn_size = int(data_model["output_size"]);
		input_seq_length = int(data_helper["input_sequence_length"]);
		output_seq_length = int(data_model["M"]) * int(data_model["G"]);
		eq_lead_time = (float) data_helper["lead_time"];
		dt = (float) data_helper["dt"];

		trim_seq_length = (int) data_helper["input_sequence_trim_length"];
		used_seq_length = input_seq_length - trim_seq_length;

		// generate observation 'X' array as a 125x2 array where column 0 is just 1's and column 1 is time corresponding to each input. Should be static.

		// TODO: add circle stuff too
		//Eigen::MatrixXf x_eq(output_chn_size);
		x_eq = Eigen::MatrixXf::Zero(output_chn_size, 1);
		Circle circle;

		timer_start = std::chrono::steady_clock::now();
	};

	//void fit(Eigen::ArrayXf current_state, Eigen::ArrayXXf input);
	//Eigen::ArrayXf getEquilibriumPoint();
	Circle circle;

	// UPDATE
	void fit(Eigen::ArrayXf current_state, Eigen::ArrayXXf input){
		// current state is ignored, only here because of compatibility with ROS wrapper
		// input is assumed to be 2x125 of [p_x, p_y] values.

		int size = used_seq_length; //input.cols();
		double x_array[size];
		double y_array[size];
		for (int i = 0; i < size; i++){
			x_array[i] = input(0, i + trim_seq_length); // x_pos
			y_array[i] = input(1, i + trim_seq_length); // y_pos
		}

		CircleData Datafitcircle(size, x_array, y_array);
		circle = CircleFitByPratt(Datafitcircle);


		x_eq(0) = circle.a + circle.r*(current_state(0)-circle.a)/(sqrt(pow(current_state(0)-circle.a,2)+pow(current_state(1)-circle.b,2)));
		x_eq(1) = circle.b + (x_eq(0)-circle.a)*(current_state(1)-circle.b)/(current_state(0)-circle.a);

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
	json params;
	int input_chn_size;
	int output_chn_size;
	int input_seq_length;
	int output_seq_length;
	int trim_seq_length;
	int used_seq_length;
	int model_order;
	float eq_lead_time;
	float dt;
	std::chrono::time_point<std::chrono::steady_clock> timer_start;
	//Eigen::FullPivHouseholderQR:::FullPivHouseholderQR<Eigen::MatrixXf> A_QR;
	Eigen::MatrixXf A;
	Eigen::MatrixXf x_eq;
	


	
};
