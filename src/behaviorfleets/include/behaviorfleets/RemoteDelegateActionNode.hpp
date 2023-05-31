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

#ifndef BEHAVIORFLEETS__REMOTEDELEGATEACTIONNODE_HPP_
#define BEHAVIORFLEETS__REMOTEDELEGATEACTIONNODE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "bf_msgs/msg/mission.hpp"
#include "behaviortree_cpp/bt_factory.h"


namespace BF
{

class RemoteDelegateActionNode : public rclcpp::Node
{
public:
  RemoteDelegateActionNode();
  RemoteDelegateActionNode(const std::string robot_id, const std::string mission_id);
  void setID(std::string id);

private:
  void mission_callback(bf_msgs::msg::Mission::UniquePtr msg);
  void mission_poll_callback(bf_msgs::msg::Mission::UniquePtr msg);
  bool create_tree();
  void control_cycle();
  void init();

  const int MAX_REQUEST_TRIES_ = 10;
  const double MAX_WAITING_TIME_ = 10.0;
  double waiting_time_ = 0.0;
  int n_tries_ = 0;
  rclcpp::Time t_last_request_;
  bf_msgs::msg::Mission::UniquePtr mission_;
  std::string id_, mission_id_;
  bool working_ = false;
  rclcpp::Publisher<bf_msgs::msg::Mission>::SharedPtr status_pub_;
  rclcpp::Publisher<bf_msgs::msg::Mission>::SharedPtr poll_pub_;
  rclcpp::Subscription<bf_msgs::msg::Mission>::SharedPtr mission_sub_;
  rclcpp::Subscription<bf_msgs::msg::Mission>::SharedPtr poll_sub_;

  BT::Tree tree_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace BF

#endif  // BEHAVIORFLEETS__REMOTEDELEGATEACTIONNODE_HPP_
