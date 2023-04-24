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

#ifndef BF_EXAMPLE__TEST_HPP_
#define BF_EXAMPLE__TEST_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "bf_msgs/msg/mission_status.hpp"
#include "bf_msgs/msg/mission_command.hpp"


namespace bf_example
{

class Test : public rclcpp::Node
{
public:
  Test();

private:
  void mission_callback(bf_msgs::msg::MissionCommand::UniquePtr msg);

  bf_msgs::msg::MissionCommand::UniquePtr mission_;

  std::string id_;
  rclcpp::Publisher<bf_msgs::msg::MissionStatus>::SharedPtr status_pub_; 
  rclcpp::Subscription<bf_msgs::msg::MissionCommand>::SharedPtr mission_sub_;
  
};

}  // namespace bf_test

#endif  // BF_EXAMPLE__TEST_HPP_
