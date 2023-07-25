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
#include <streambuf>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_cpp/utils/safe_any.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Read node parameters
  auto node = rclcpp::Node::make_shared("source_tree");
  node->declare_parameter("plugins", std::vector<std::string>({}));
  node->declare_parameter("behavior_tree_xml", "");

  std::vector<std::string> plugins;
  std::string bt_xml_filename;
  node->get_parameter("plugins", plugins);
  node->get_parameter("behavior_tree_xml", bt_xml_filename);

  if (bt_xml_filename == "") {
    RCLCPP_ERROR(node->get_logger(), "BT XML Filename void");
    return -1;
  }

  // Loading plugins from parameters
  BT::SharedLibrary loader;
  BT::BehaviorTreeFactory factory;

  RCLCPP_INFO_STREAM(node->get_logger(), "Loading " << plugins.size() << " plugins" << ":");
  for (const auto & plugin : plugins) {
    RCLCPP_INFO_STREAM(node->get_logger(), "\t" << plugin);
    factory.registerFromPlugin(loader.getOSName(plugin));
  }

  // Read Behavior Tree XML file into an string
  std::string pkgpath = ament_index_cpp::get_package_share_directory("behaviorfleets");
  std::string xml_file = pkgpath + "/bt_xml/" + bt_xml_filename;
  RCLCPP_INFO_STREAM(node->get_logger(), "Loading " << xml_file);
  std::ifstream ifs_xml(xml_file);
  std::string str_xml(
    (std::istreambuf_iterator<char>(ifs_xml)),
    std::istreambuf_iterator<char>());

  // Create Tree from XML readed and init blackboard, inserting XML on it to be used from BT nodes
  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  blackboard->set("bt_xml", str_xml);

  BT::Tree tree = factory.createTreeFromText(str_xml, blackboard);

  // Execute tree
  rclcpp::Rate rate(10);
  bool finish = false;
  while (!finish && rclcpp::ok()) {
    finish = tree.rootNode()->executeTick() != BT::NodeStatus::RUNNING;
    rclcpp::spin_some(node);
    rate.sleep();
  }

  RCLCPP_INFO_STREAM(node->get_logger(), "Finished ");
  rclcpp::shutdown();
  return 0;
}
