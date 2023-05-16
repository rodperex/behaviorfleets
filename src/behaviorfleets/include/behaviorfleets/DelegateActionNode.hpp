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


#ifndef BEHAVIORFLEETS__DELEGATEACTIONNODE_HPP_
#define BEHAVIORFLEETS__DELEGATEACTIONNODE_HPP_

#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/decorator_node.h"

#include "bf_msgs/msg/mission_command.hpp"
#include "bf_msgs/msg/mission_status.hpp"


#include "rclcpp/rclcpp.hpp"

namespace BF
{

class DelegateActionNode : public BT::ActionNodeBase
{
public:
  DelegateActionNode(
    const std::string& name,
    const BT::NodeConfig& conf);


  void remote_status_callback(bf_msgs::msg::MissionStatus::UniquePtr msg);

  void halt() override
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("remote_tree"),
      BT::InputPort<char*>("remote_id"),
    };
  }

private:
  static constexpr const char* MISSION = "";

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<bf_msgs::msg::MissionCommand>::SharedPtr tree_pub_;
  rclcpp::Subscription<bf_msgs::msg::MissionStatus>::SharedPtr remote_sub_;

  bf_msgs::msg::MissionStatus::UniquePtr remote_status_;
  std::string remote_id_, remote_tree_;
  bool remote_identified_ = false;

  static const int FAILURE = 0;
  static const int SUCCESS = 1;
  static const int RUNNING = 2;

  BT::NodeStatus tick() override;
};

}   // namespace BF

#endif  // BEHAVIORFLEETS__DELEGATEACTIONNODE_HPP_
