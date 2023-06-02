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

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_cpp/utils/safe_any.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"

#include "yaml-cpp/yaml.h"

bool is_pointer(const std::string & type_name);
std::vector<std::string> check_blackboard(BT::Blackboard::Ptr bb, BT::Blackboard::Ptr bb_cache);

int main(int argc, char * argv[])
{
  std::string params_file = "config.yaml";

  if (argc > 1) {
    params_file = std::string(argv[1]);
  }

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("source_tree");

  BT::SharedLibrary loader;
  BT::BehaviorTreeFactory factory;

  factory.registerFromPlugin(loader.getOSName("delegate_action_node"));

  std::string pkgpath = ament_index_cpp::get_package_share_directory("behaviorfleets");

  std::string xml_file;

  try {
    // Load the XML path from the YAML file
    std::cout << "Configuration file: " << params_file << std::endl;
    std::ifstream fin(pkgpath + "/params/" + params_file);
    YAML::Node params = YAML::Load(fin);

    xml_file = pkgpath + params["source_tree"].as<std::string>();

  } catch (YAML::Exception & e) {
    std::cerr << "Error loading YAML file: " << e.what() << std::endl;
    return 1;
  } catch (std::exception & e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  blackboard->set("pkgpath", pkgpath + "/bt_xml/");

  blackboard->set("double", 33.22);
  float * b = new float(20);
  blackboard->set("b", b);
  char * c = new char('c');
  blackboard->set("c", c);
  blackboard->set("entero", 3);

  auto bb_cache = BT::Blackboard::create();
  bb_cache->set("pkgpath", blackboard->get<std::string>("pkgpath"));
  bb_cache->set("double", 33.22);
  bb_cache->set("b", b);
  bb_cache->set("c", c);
  bb_cache->set("entero", 3);

  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  std::cout << "\t- Tree created from file" << std::endl;

  rclcpp::Rate rate(10);

  bool finish = false;
  while (!finish && rclcpp::ok()) {
    finish = tree.rootNode()->executeTick() != BT::NodeStatus::RUNNING;
    rclcpp::spin_some(node);
    rate.sleep();

  }

  std::cout << "Finished" << std::endl;
  rclcpp::shutdown();
  return 0;
}

