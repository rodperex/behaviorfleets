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

#ifndef BEHAVIORFLEETS__BLACKBOARDSTRESSER_HPP_
#define BEHAVIORFLEETS__BLACKBOARDSTRESSER_HPP_

#include <random>
#include <chrono>
#include <fstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "behaviorfleets/BlackboardHandler.hpp"

#include "behaviortree_cpp/blackboard.h"

namespace BF
{

class BlackboardStresser : public rclcpp::Node
{
public:
  BlackboardStresser(
    const std::string robot_id, const int n_keys,
    std::chrono::milliseconds milis,
    std::chrono::seconds op_time,
    std::chrono::seconds delay);

private:
  void control_cycle();
  // void bb_handler_spinner();
  void update_blackboard();
  void dump_blackboard();
  int random_int(int min, int max);

  std::thread spin_thread_;

  std::string robot_id_;

  std::vector<std::string> keys_;

  BT::Blackboard::Ptr blackboard_;

  BF::BlackboardHandler::SharedPtr bb_handler_;

  rclcpp::TimerBase::SharedPtr timer_, bb_handler_timer_;

  rclcpp::Time t_start_;

  std::chrono::seconds op_time_, delay_;

  int n_changes_;

  bool bb_handler_spinning_;
};

}   // namespace BF

#endif  // BEHAVIORFLEETS__BLACKBOARDSTRESSER_HPP_
