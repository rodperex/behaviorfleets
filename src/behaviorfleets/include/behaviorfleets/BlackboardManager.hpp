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

#ifndef BEHAVIORFLEETS__BLACKBOARDMANAGER_HPP_
#define BEHAVIORFLEETS__BLACKBOARDMANAGER_HPP_

#include <string>
#include <queue>

#include "behaviortree_cpp/blackboard.h"

#include "bf_msgs/msg/blackboard.hpp"

#include "rclcpp/rclcpp.hpp"

namespace BF
{

class BlackboardManager : public rclcpp::Node
{
public:
  BlackboardManager();
  
private:
  void blackboard_callback(bf_msgs::msg::Mission::Blackboard msg);
  void control_cycle();
  void grant_bb();

  BT::Blackboard::Ptr blackboard_;
  // bf_msgs::msg::Blackboard bb_msg_;
  bool lock_;
  std::string robot_id_;
  std::queue<std::string> q_;

  rclcpp::Publisher<bf_msgs::msg::Blackboard>::SharedPtr bb_pub_;
  rclcpp::Subscription<bf_msgs::msg::Blackboard>::SharedPtr bb_sub_;
};

}   // namespace BF

#endif  // BEHAVIORFLEETS__BLACKBOARDMANAGER_HPP_
