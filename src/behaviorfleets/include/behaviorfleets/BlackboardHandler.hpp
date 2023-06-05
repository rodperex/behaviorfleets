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

#ifndef BEHAVIORFLEETS__BLACKBOARDHANDLER_HPP_
#define BEHAVIORFLEETS__BLACKBOARDHANDLER_HPP_

#include <string>

#include "behaviortree_cpp/blackboard.h"

#include "bf_msgs/msg/blackboard.hpp"

#include "rclcpp/rclcpp.hpp"

namespace BF
{

class BlackboardHandler : public rclcpp::Node
{
public:
  BlackboardHandler(const std::string robot_id, BT::Blackboard::Ptr blackboard); // bb_cache_ = BT::Blackboard(blackboard);
  
private:
  void blackboard_callback(bf_msgs::msg::Mission::Blackboard msg);
  void control_cycle();
  void propagate_bb_update();

  BT::Blackboard::Ptr blackboard_;
  BT::Blackboard bb_cache_;
  std::string robot_id_;

  rclcpp::Publisher<bf_msgs::msg::Blackboard>::SharedPtr bb_pub_;
  rclcpp::Subscription<bf_msgs::msg::Blackboard>::SharedPtr bb_sub_;
};

}   // namespace BF

#endif  // BEHAVIORFLEETS__BLACKBOARDHANDLER_HPP_
