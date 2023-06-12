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
#include <fstream>
#include <chrono>

#include "behaviortree_cpp/blackboard.h"

#include "rclcpp/rclcpp.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "yaml-cpp/yaml.h"

#include "behaviorfleets/BlackboardManager.hpp"

int main(int argc, char * argv[])
{
  std::string params_file = "stress_test_config.yaml";

  if (argc > 1) {
    params_file = std::string(argv[1]);
  }

  rclcpp::init(argc, argv);

  std::string pkgpath = ament_index_cpp::get_package_share_directory("behaviorfleets");

  try {
    // Load the XML path from the YAML file
    std::cout << "Configuration file: " << params_file << std::endl;
    std::ifstream fin(pkgpath + "/params/" + params_file);
    YAML::Node params = YAML::Load(fin);

    float freq = params["manager_hz"].as<float>();

    auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<float, std::milli>((1 / freq) * 1000)
    );

    auto blackboard = BT::Blackboard::create();

    auto bb_manager = std::make_shared<BF::BlackboardManager>(blackboard, period);
    // auto bb_manager = std::make_shared<BF::BlackboardManager>(blackboard);

    rclcpp::spin(bb_manager);

    std::cout << "Finished" << std::endl;
    rclcpp::shutdown();
    return 0;

  } catch (YAML::Exception & e) {
    std::cerr << "Error loading YAML file: " << e.what() << std::endl;
    return 1;
  } catch (std::exception & e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }
}
