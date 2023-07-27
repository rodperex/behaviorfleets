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
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <random>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/decorator_node.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "ament_index_cpp/get_package_share_directory.hpp"


#include "bf_msgs/msg/mission.hpp"

#include "rclcpp/rclcpp.hpp"

namespace BF
{

class DelegateActionNode : public BT::ActionNodeBase
{
public:
  DelegateActionNode(
    const std::string & name,
    const BT::NodeConfig & conf);


  void remote_status_callback(bf_msgs::msg::Mission::UniquePtr msg);
  void mission_poll_callback(bf_msgs::msg::Mission::UniquePtr msg);
  void set_name();

  void halt() override
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("mission_id"),
      BT::InputPort<std::string>("remote_tree"),
      BT::InputPort<std::string>("remote_id"),
      BT::InputPort<std::string>("exclude"),
      BT::InputPort<std::string>("plugins"),
      BT::InputPort<double>("timeout"),
      BT::InputPort<int>("max_tries")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<bf_msgs::msg::Mission>::SharedPtr mission_pub_;
  rclcpp::Publisher<bf_msgs::msg::Mission>::SharedPtr poll_pub_;
  rclcpp::Subscription<bf_msgs::msg::Mission>::SharedPtr remote_sub_;
  rclcpp::Subscription<bf_msgs::msg::Mission>::SharedPtr poll_sub_;
  void decode(std::string str, std::vector<std::string> * vector);
  bool is_remote_excluded(std::string remote_id);
  void reset();

  bf_msgs::msg::Mission::UniquePtr remote_status_, poll_answ_;
  std::string remote_id_, remote_tree_, mission_id_, me_;
  std::vector<std::string> plugins_, excluded_;
  bool remote_identified_ = false;
  bool read_tree_from_port_;
  rclcpp::Time t_last_status_, t_last_poll_;
  double timeout_, poll_timeout_;
  int MAX_TRIES_, n_tries_ = 0;
  int tick_count_ = 0;

  BT::NodeStatus tick() override;
};

}   // namespace BF

#endif  // BEHAVIORFLEETS__DELEGATEACTIONNODE_HPP_
