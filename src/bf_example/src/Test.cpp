// Copyright 2021 Intelligent Robotics Lab
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
#include <iostream>


#include "Test.hpp"

#include "behaviortree_cpp/behavior_tree.h"
#include "bf_msgs/msg/mission_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bf_example
{

Test::Test()
: Node("test"),
id_("test")
{
  mission_sub_ = create_subscription<bf_msgs::msg::MissionCommand>(
    "/mission_command", rclcpp::SensorDataQoS(),
    std::bind(&Test::mission_callback, this, std::placeholders::_1));

  status_pub_ = create_publisher<bf_msgs::msg::MissionStatus>("/mission_status", 10);

}

void mission_callback(bf_msgs::msg::MissionCommand::UniquePtr msg){
  mission_ = std::move(msg);
}

}  // namespace bf_example
