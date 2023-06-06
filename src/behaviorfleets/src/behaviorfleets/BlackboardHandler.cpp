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
  blackboard_(blackboard),
  access_granted_(false),
  request_sent_(false)
{
  using namespace std::chrono_literals;

  bb_cache_ = BT::Blackboard::create();
  cache_blackboard();

  bb_pub_ = create_publisher<bf_msgs::msg::Blackboard>(
    "/blackboard", 100);

  bb_sub_ = create_subscription<bf_msgs::msg::Blackboard>(
    "/blackboard", rclcpp::SensorDataQoS(),
    std::bind(&BlackboardHandler::blackboard_callback, this, std::placeholders::_1));

  timer_ = create_wall_timer(50ms, std::bind(&BlackboardHandler::control_cycle, this));
}

void BlackboardHandler::control_cycle()
{
  if (has_bb_changed()) {
    update_blackboard();
    cache_blackboard();
  }
}

bool BlackboardHandler::has_bb_changed()
{
  std::vector<BT::StringView> sv_bb = blackboard_->getKeys();
  std::vector<BT::StringView> sv_cache_bb = bb_cache_->getKeys();

  for (const auto & entry_bb : sv_bb) {
    if (std::find(
        excluded_keys_.begin(), excluded_keys_.end(),
        entry_bb.data()) != excluded_keys_.end())
    {
      // the key is in the exclusion list
      continue;
    }
    if (std::find(sv_cache_bb.begin(), sv_cache_bb.end(), entry_bb) == sv_cache_bb.end()) {
      // the key is not in the cache
      RCLCPP_INFO(get_logger(), "Key %s not in cache", entry_bb.data());
      return true;
    } else if (blackboard_->get<std::string>(entry_bb.data()) !=
      bb_cache_->get<std::string>(entry_bb.data()))
    {
      RCLCPP_INFO(get_logger(), "Key %s has changed", entry_bb.data());
      return true;
    }
  }
  return false;
}

void BlackboardHandler::blackboard_callback(bf_msgs::msg::Blackboard::UniquePtr msg)
{
  if ((msg->type == bf_msgs::msg::Blackboard::GRANT) && (msg->robot_id == robot_id_)) {
    RCLCPP_INFO(get_logger(), "Access to blackboard granted");
    access_granted_ = true;
    return;
  }
  if ((msg->type == bf_msgs::msg::Blackboard::PUBLISH) && (msg->robot_id == "all")) {
    RCLCPP_INFO(get_logger(), "Updating local blackboard from the shared one");
    for (int i = 0; i < msg->keys.size(); i++) {
      blackboard_->set(msg->keys[i], msg->values[i]);
    }
    cache_blackboard();
    return;
  }
  if ((msg->type == bf_msgs::msg::Blackboard::DENY) && (msg->robot_id != robot_id_)) {
    RCLCPP_INFO(get_logger(), "Access to blackboard denied");
    request_sent_ = false;
    return;
  }
}

void BlackboardHandler::cache_blackboard()
{
  bb_cache_->clear();

  std::vector<BT::StringView> string_views = blackboard_->getKeys();
  for (const auto & string_view : string_views) {
    try {
      bb_cache_->set(string_view.data(), blackboard_->get<std::string>(string_view.data()));
      RCLCPP_INFO(get_logger(), "Key %s cached", string_view.data());
    } catch (const std::exception & e) {
      excluded_keys_.push_back(string_view.data());
      RCLCPP_INFO(get_logger(), "Key %s skipped", string_view.data());
    }
  }
}

void BlackboardHandler::update_blackboard()
{
  bf_msgs::msg::Blackboard msg;

  if (access_granted_) {
    RCLCPP_INFO(get_logger(), "Access to blackboard granted");
    std::vector<BT::StringView> string_views = blackboard_->getKeys();  
    int i = 0;
    msg.robot_id = robot_id_;
    msg.type = bf_msgs::msg::Blackboard::UPDATE;
    for (const auto & string_view : string_views) {
      if (std::find(
          excluded_keys_.begin(), excluded_keys_.end(),
          string_view.data()) == excluded_keys_.end())
      {
        msg.keys[i] = string_view.data();
        msg.values[i] = blackboard_->get<std::string>(string_view.data());
        i++;
      }
    }
    bb_pub_->publish(msg);
    request_sent_ = false;
    access_granted_ = false;
  } else {
    RCLCPP_INFO(get_logger(), "Requesting access to blackboard");
    msg.type = bf_msgs::msg::Blackboard::REQUEST;
    if (!request_sent_) {
      bb_pub_->publish(msg);
      request_sent_ = true;
    } else {
      RCLCPP_INFO(get_logger(), "Waiting for access to blackboard");
    }
  }
}

}  // namespace BF
