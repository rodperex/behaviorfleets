// Copyright 2023 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <memory>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "yaml-cpp/yaml.h"

#include "test/BlackboardStresser.hpp"

double random_double(double min, double max);
int random_int(int min, int max);

int main(int argc, char * argv[])
{

  using namespace std::chrono_literals;
  std::string params_file = "stress_test_config.yaml";

  if (argc > 1) {
    params_file = std::string(argv[1]);
  }

  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;

  std::string pkgpath = ament_index_cpp::get_package_share_directory("behaviorfleets");

  std::list<std::shared_ptr<BF::BlackboardStresser>> nodes;

  try {
    std::cout << "Configuration file: " << params_file << std::endl;
    std::ifstream fin(pkgpath + "/params/" + params_file);
    YAML::Node params = YAML::Load(fin);

    int num_nodes = params["nodes"].as<int>();
    int n_keys = params["n_keys"].as<int>();
    std::chrono::seconds op_time(params["op_time"].as<int>());
    int max_delay = params["max_delay"].as<int>();
    float freq = params["stresser_hz"].as<float>();
    double max_dev = params["dev_hz"].as<double>();

    for (int i = 0; i < num_nodes; ++i) {

      freq = freq + random_double(0.0, max_dev);
      int half = random_int(1, 2);
      auto delay = std::chrono::seconds(random_int(0, max_delay / half));

      auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<float, std::milli>((1 / freq) * 1000)
      );

      std::string name = "bb_stresser_" + std::to_string(i + 1);
      auto node = std::make_shared<BF::BlackboardStresser>(name, n_keys, period, op_time, delay);
      nodes.push_back(node);
      exec.add_node(node);
    }

  } catch (YAML::Exception & e) {
    std::cerr << "Error loading YAML file: " << e.what() << std::endl;
    return 1;
  }

  exec.spin();

  rclcpp::shutdown();
  return 0;
}

double random_double(double min, double max)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(min, max);

  return dis(gen);
}

int random_int(int min, int max)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(min, max);

  return dis(gen);
}
