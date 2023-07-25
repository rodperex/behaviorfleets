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

#include "rclcpp/rclcpp.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviorfleets/RemoteDelegateActionNode.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  std::string pkgpath = ament_index_cpp::get_package_share_directory("behaviorfleets");
  std::list<std::shared_ptr<BF::RemoteDelegateActionNode>> nodes;

  auto node_aux = rclcpp::Node::make_shared("remote_tree");
  node_aux->declare_parameter("behavior_tree_xml", "");
  node_aux->declare_parameter("nodes", 0);
  node_aux->declare_parameter("missions", std::vector<std::string>({}));

  std::string source_tree;
  int num_nodes;
  std::vector<std::string> missions;
  node_aux->get_parameter("source_tree", source_tree);
  node_aux->get_parameter("nodes", num_nodes);
  node_aux->get_parameter("missions", missions);

  int mission_index = 0;
  for (int i = 0; i < num_nodes; ++i) {
    std::string name = "dummy" + std::to_string(i + 1);
    std::string type = missions[mission_index];

    auto node = std::make_shared<BF::RemoteDelegateActionNode>(name, type);
    nodes.push_back(node);
    exec.add_node(node);

    RCLCPP_INFO_STREAM(
      node_aux->get_logger(), 
      "\n\n******** Created node " << name << " with mission " << type << "\n\n");

    mission_index = (mission_index + 1) % missions.size();
  }

  exec.spin();

  rclcpp::shutdown();
  return 0;
}
