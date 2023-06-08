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
  BT::Blackboard::Ptr blackboard)
: Node("blackboard_manager")
{
  init();
  copy_blackboard(blackboard);
}

BlackboardManager::BlackboardManager(
  BT::Blackboard::Ptr blackboard,
  std::chrono::milliseconds milis)
: Node("blackboard_manager")
{
  init();
  copy_blackboard(blackboard);

  timer_publish_ = create_wall_timer(milis, std::bind(&BlackboardManager::publish_blackboard, this));
}

void BlackboardManager::init()
{
  using::std::chrono_literals::operator""ms;
  
  lock_ = false;
  robot_id_ = "";

  blackboard_ = BT::Blackboard::create();

  bb_pub_ = create_publisher<bf_msgs::msg::Blackboard>(
    "/blackboard", 100);

  bb_sub_ = create_subscription<bf_msgs::msg::Blackboard>(
    "/blackboard", rclcpp::SensorDataQoS(),
    std::bind(&BlackboardManager::blackboard_callback, this, std::placeholders::_1));

  timer_cycle_ = create_wall_timer(50ms, std::bind(&BlackboardManager::control_cycle, this));
}

void BlackboardManager::control_cycle()
{
  if (!lock_ && !q_.empty()) {
    RCLCPP_INFO(get_logger(), "dequeuing robot %s (%zu pending)", q_.front().c_str(), q_.size() - 1);
    robot_id_ = q_.front();
    q_.pop();
    grant_blackboard();
  }
}

void BlackboardManager::grant_blackboard()
{
  RCLCPP_INFO(get_logger(), "granting blackboard to [%s]", robot_id_.c_str());
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
    // blackboard locked by other robot
    if ((robot_id_ != update_bb_msg_->robot_id) &&
      (update_bb_msg_->type == bf_msgs::msg::Blackboard::REQUEST))
    {
      // equeue request
      q_.push(update_bb_msg_->robot_id);
      RCLCPP_INFO(get_logger(), "request from robot %s enqueued", update_bb_msg_->robot_id.c_str());
    } else if (update_bb_msg_->type == bf_msgs::msg::Blackboard::UPDATE) {
      update_blackboard();
    }
  } else if (update_bb_msg_->type == bf_msgs::msg::Blackboard::REQUEST) {
    // blackboard free
    RCLCPP_INFO(get_logger(), "blackboard free");
    if (q_.empty()) { // prioritize to earlier requests
      RCLCPP_INFO(get_logger(), "no pending requests");
      robot_id_ = update_bb_msg_->robot_id;
      grant_blackboard();
    } else {
      RCLCPP_INFO(get_logger(), "request from robot %s enqueued", update_bb_msg_->robot_id.c_str());
      q_.push(update_bb_msg_->robot_id);
    }
    
  }
}

void BlackboardManager::update_blackboard()
{
  RCLCPP_INFO(get_logger(), "robot %s updating blackboard", robot_id_.c_str());

  std::vector<std::string> keys = update_bb_msg_->keys;
  std::vector<std::string> values = update_bb_msg_->values;

  blackboard_->clear();
  for (int i = 0; i < keys.size(); i++) {
    blackboard_->set(keys[i], values[i]);
  }

  lock_ = false;

  publish_blackboard();
}

void BlackboardManager::publish_blackboard()
{
  RCLCPP_INFO(get_logger(), "publishing blackboard");

  // while (lock_) {
  //   std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // }

  if (!lock_) {
    lock_ = true;
    bf_msgs::msg::Blackboard msg;
    std::vector<BT::StringView> string_views = blackboard_->getKeys();

    msg.type = bf_msgs::msg::Blackboard::PUBLISH;
    // msg.robot_id = "all";
    msg.robot_id = robot_id_;
    std::vector<std::string> keys;
    std::vector<std::string> values;

    for (const auto & string_view : string_views) {
      try {
        keys.push_back(string_view.data());
        values.push_back(blackboard_->get<std::string>(string_view.data()));
      } catch (const std::exception & e) {
        RCLCPP_INFO(get_logger(), "key %s skipped", string_view.data());
      }
    }
    msg.keys = keys;
    msg.values = values;  
    bb_pub_->publish(msg);
    lock_ = false;
    robot_id_ = "";
  }
}

void BlackboardManager::copy_blackboard(BT::Blackboard::Ptr source_bb)
{
  blackboard_->clear();

  std::vector<BT::StringView> string_views = source_bb->getKeys();
  for (const auto & string_view : string_views) {
    try {
      blackboard_->set(string_view.data(), blackboard_->get<std::string>(string_view.data()));
      RCLCPP_INFO(get_logger(), "key %s copied", string_view.data());
    } catch (const std::exception & e) {
      RCLCPP_INFO(get_logger(), "key %s skipped", string_view.data());
    }
  }
}

}  // namespace BF
