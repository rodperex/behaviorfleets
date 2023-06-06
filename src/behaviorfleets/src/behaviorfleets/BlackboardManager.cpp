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

#include "behaviorfleets/BlackboardManager.hpp"

#include "behaviortree_cpp/blackboard.h"
#include "bf_msgs/msg/blackboard.hpp"

namespace BF
{

BlackboardManager::BlackboardManager()
  : Node("blackboard_manager")
{
  init();
}

BlackboardManager::BlackboardManager(
  std::chrono::milliseconds milis)
  : Node("blackboard_manager")
{
  init();

  timer_ = create_wall_timer(milis, std::bind(&BlackboardManager::propagate_bb, this));
}

void BlackboardManager::init()
{
  lock_ = false;
  robot_id_ = "";
    
  bb_pub_ = create_publisher<bf_msgs::msg::Blackboard>(
    "/shared_bb", 100);
  
  bb_sub_ = create_subscription<bf_msgs::msg::Blackboard>(
    "/shared_bb", rclcpp::SensorDataQoS(),
    std::bind(&BlackboardManager::blackboard_callback, this, std::placeholders::_1));
}

void BlackboardManager::control_cycle()
{
  if (!lock_ && !q_.empty()) {
    robot_id_ = q_.front();
    q_.pop();
    grant_bb();
  }
}

void BlackboardManager::grant_bb() {
  bf_msgs::msg::Blackboard answ;
  answ.type = bf_msgs::msg::Blackboard::GRANT;
  answ.robot_id = robot_id_;
  bb_pub_->publish(answ);
  lock_ = true;
}

void BlackboardManager::blackboard_callback(bf_msgs::msg::Blackboard::UniquePtr msg)
{
  update_bb_msg_ = std::move(msg);
  bf_msgs::msg::Blackboard answ;

  if (lock_) {
    if ((robot_id_ != update_bb_msg_->robot_id) && (update_bb_msg_->type == bf_msgs::msg::Blackboard::REQUEST)) {
      q_.push(update_bb_msg_->robot_id);
    } else if (update_bb_msg_->type == bf_msgs::msg::Blackboard::UPDATE) {
        // update the blackboard
    }
  } else if (update_bb_msg_->type == bf_msgs::msg::Blackboard::REQUEST) {
      robot_id_ = update_bb_msg_->robot_id;
      grant_bb();
  }
}

void BlackboardManager::update_bb()
{
  RCLCPP_INFO(get_logger(), "Robot %s updating blackboard", robot_id_.c_str());
  
  std::vector<std::string> keys = update_bb_msg_->keys;
  std::vector<std::string> values = update_bb_msg_->values;

  for (int i = 0; i < keys.size(); i++) {
    blackboard_->set(keys[i], values[i]);
  }
}

void BlackboardManager::propagate_bb()
{

}

}  // namespace BF
