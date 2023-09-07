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
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp/blackboard.h"

#include "bf_msgs/msg/blackboard.hpp"

namespace BF
{

class BlackboardHandler : public rclcpp::Node
{
public:
  BlackboardHandler(const std::string robot_id, BT::Blackboard::Ptr blackboard);
  // BlackboardHandler(const std::string robot_id, BT::Blackboard::Ptr blackboard, std::chrono::milliseconds milis);
  virtual ~BlackboardHandler();

private:
  void blackboard_callback(bf_msgs::msg::Blackboard::UniquePtr msg);
  std::string get_type(const char * port_name);
  void control_cycle();
  void update_blackboard();
  void cache_blackboard();
  bool has_bb_changed();
  void dump_data();
  void sync_bb();


  BT::Blackboard::Ptr blackboard_, bb_cache_;
  std::string robot_id_;
  std::vector<std::string> excluded_keys_;
  bool access_granted_, request_sent_;
  rclcpp::Time t_last_request_;

  rclcpp::Publisher<bf_msgs::msg::Blackboard>::SharedPtr bb_pub_;
  rclcpp::Subscription<bf_msgs::msg::Blackboard>::SharedPtr bb_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  bool sync_rcvd_;

  // test stuff
  rclcpp::Time waiting_time_;
  double avg_waiting_time_;
  int n_success_, n_requests_, n_updates_;
};

}   // namespace BF

#endif  // BEHAVIORFLEETS__BLACKBOARDHANDLER_HPP_
