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
#include <chrono>
#include <fstream>
#include <iostream>
#include <cxxabi.h>
#include <cstdlib>

#include "behaviortree_cpp/blackboard.h"

#include "bf_msgs/msg/blackboard.hpp"

#include "rclcpp/rclcpp.hpp"

namespace BF
{

class BlackboardManager : public rclcpp::Node
{
public:
  BlackboardManager();
  BlackboardManager(BT::Blackboard::Ptr blackboard);
  BlackboardManager(BT::Blackboard::Ptr blackboard, std::chrono::milliseconds milis);
  BlackboardManager(
    BT::Blackboard::Ptr blackboard, std::chrono::milliseconds milis,
    std::chrono::milliseconds bb_refresh_rate);

private:
  void blackboard_callback(bf_msgs::msg::Blackboard::UniquePtr msg);
  void copy_blackboard(BT::Blackboard::Ptr source_bb);
  std::string get_type(const char * port_name);
  void init();
  void control_cycle();
  void grant_blackboard();
  void update_blackboard();
  void publish_blackboard();
  void dump_blackboard();
  void dump_waiting_times();

  bf_msgs::msg::Blackboard::UniquePtr update_bb_msg_;
  BT::Blackboard::Ptr blackboard_;
  bool lock_;
  std::string robot_id_;
  std::queue<std::string> q_;

  rclcpp::Time t_last_grant_;

  rclcpp::Publisher<bf_msgs::msg::Blackboard>::SharedPtr bb_pub_;
  rclcpp::Subscription<bf_msgs::msg::Blackboard>::SharedPtr bb_sub_;

  rclcpp::TimerBase::SharedPtr timer_publish_, timer_cycle_;

  std::queue<rclcpp::Time> q_start_wait_;
  std::vector<rclcpp::Duration> waiting_times_;
  int tam_q_, n_pub_;
};

} // namespace BF

#endif // BEHAVIORFLEETS__BLACKBOARDMANAGER_HPP_
