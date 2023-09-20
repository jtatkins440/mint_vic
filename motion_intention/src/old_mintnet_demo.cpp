//#include <torch/script.h> // One-stop header.
//#include <nlohmann/json.hpp>
//#include <Eigen/Dense>
#include "mintwrapper.h"

#include <iostream>
#include <fstream>
#include <memory>
#include <deque>

using json = nlohmann::json;
using namespace torch::indexing;

int main(int argc, const char* argv[]) {
  if (argc != 2) {
    std::cerr << "usage: example-app <path-to-exported-script-module>\n";
    return -1;
  }

  std::string hparams = "hyper_params.json"; // hardcoding is bad! Fix it!

  MIntWrapper minty(argv[1], hparams);
  
  std::ifstream f(hparams);
  json data = json::parse(f);
  std::cout << data << "\n";

  json data_helper = data["helper_params"];
  json data_model = data["mdl_params"];
  const int input_chn_size = int(data_model["input_size"]);
  const int output_chn_size = int(data_model["output_size"]);
  const int input_seq_length = int(data_helper["input_sequence_length"]);
  const int output_seq_length = int(data_model["M"]) * int(data_model["G"]);

  // begin simulation
  
  double max_sim_time = 3.0;
  double sample_rate = 0.005;
  auto sim_timer_start = std::chrono::steady_clock::now();
  auto sample_timer_start = std::chrono::steady_clock::now();
  //while ((double) (std::chrono::steady_clock::now() - sim_timer_start) <  max_sim_time)
  //{ }

  // Create a vector of inputs.
  std::vector<torch::jit::IValue> inputs;

  Eigen::ArrayXXf input_a = Eigen::ArrayXXf::Ones(input_chn_size, input_seq_length);

  std::cout << "ones input: " << input_a << std::endl;

  // test to see if forward works correctly
  auto output_minty = minty.forward(input_a);
  std::cout << "got it!" << std::endl;
  std::cout << output_minty << std::endl;

  // test to see if spline fitting plus forward works
  Eigen::ArrayXf current_state(6,1);
  current_state << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
  minty.forwardFitSpline(current_state, input_a);
  std::cout << "fit spline!" << std::endl;

  // test to see if the internal timer is working correctly
  for (int i = 0; i < 20; i++)
  {
    auto output_eq = minty.getEquilibriumPoint();
    std::cout << "got eq!" << std::endl;
    std::cout << output_eq << std::endl;
  }
  

  // test the deque
  std::deque<int> test_deque;
  int max_deque_size = 10;
  for (int i = 0; i < 100; i++)
  {
    if (test_deque.size() >= max_deque_size) {test_deque.pop_front();}
    test_deque.push_back(i);
    if (i % 10 == 0) { for (int n : test_deque) {std::cout << n << ", ";} std::cout << std::endl;}
  }
  for (int n : test_deque) {std::cout << n << ", ";} std::cout << std::endl;
}

