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

#include "behaviorfleets/BlackboardHandler.hpp"

#include "behaviortree_cpp/blackboard.h"
#include "bf_msgs/msg/blackboard.hpp"

namespace BF
{

BlackboardHandler::BlackboardHandler(
  const std::string robot_id,
  BT::Blackboard::Ptr blackboard)
  : Node("blackboard_handler"),
  robot_id_(robot_id),
  blackboard_(blackboard)
{
  bb_cache_ = BT::Blackboard(*blackboard_);
  
  bb_pub_ = create_publisher<bf_msgs::msg::Blackboard>(
    "/shared_bb", 100);
  
  bb_sub_ = create_subscription<bf_msgs::msg::Blackboard>(
    "/shared_bb", rclcpp::SensorDataQoS(),
    std::bind(&BlackboardManager::blackboard_callback, this, std::placeholders::_1));     
}

void control_cycle()
{

}


void BlackboardManager::blackboard_callback(bf_msgs::msg::Blackboard msg)
{

}

}  // namespace BF
