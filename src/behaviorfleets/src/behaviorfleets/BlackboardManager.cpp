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
  
  rclcpp::on_shutdown([this]() {dump_blackboard();});
}

void BlackboardManager::control_cycle()
{
  if (!lock_ && !q_.empty()) {
    RCLCPP_INFO(get_logger(), "dequeuing robot %s (%zu pending)", q_.front().c_str(), q_.size() - 1);
    robot_id_ = q_.front();
    q_.pop();
    grant_blackboard();
  } else if ((rclcpp::Clock().now() - t_last_grant_).seconds() > 5.0) {
    // RCLCPP_INFO(get_logger(), "blackboard free");
    robot_id_ = "";
    lock_ = false;
  }
}

void BlackboardManager::grant_blackboard()
{
  t_last_grant_ = rclcpp::Clock().now();
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

  if (update_bb_msg_->type == bf_msgs::msg::Blackboard::REQUEST) {
    // enqueue all requests
    q_.push(update_bb_msg_->robot_id);
    RCLCPP_INFO(get_logger(), "request from robot %s enqueued", update_bb_msg_->robot_id.c_str());
  } else if ((update_bb_msg_->type == bf_msgs::msg::Blackboard::UPDATE) &&
    (update_bb_msg_->robot_id == robot_id_)) {
      // attend update request coming from the robot that has the blackboard
      update_blackboard();
  }
}

void BlackboardManager::update_blackboard()
{
  RCLCPP_INFO(get_logger(), "robot %s updating blackboard", robot_id_.c_str());

  std::vector<std::string> keys = update_bb_msg_->keys;
  std::vector<std::string> values = update_bb_msg_->values;

  // blackboard_->clear();
  // manager cannot clear the blackboard, only update it
  for (int i = 0; i < keys.size(); i++) {
    blackboard_->set(keys[i], values[i]);
  }

  lock_ = false;

  publish_blackboard();
}

void BlackboardManager::publish_blackboard()
{
  // RCLCPP_INFO(get_logger(), "publishing blackboard");

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

    RCLCPP_INFO(get_logger(), "blackboard published");
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

void BlackboardManager::dump_blackboard()
{
  RCLCPP_INFO(get_logger(), "dumping blackboard");
  std::string filename = "src/behaviorfleets/results/manager.txt";
  std::ofstream file(filename,std::ofstream::out);

  std::vector<std::pair<std::string, std::string>> kv_pairs;
  std::vector<BT::StringView> string_views = blackboard_->getKeys();
  
  for (const auto & string_view : string_views) {
    kv_pairs.push_back(std::make_pair(string_view.data(),
      blackboard_->get<std::string>(string_view.data())));
  }

  std::sort(kv_pairs.begin(), kv_pairs.end(),
    [](const std::pair<std::string, std::string>& a,
    const std::pair<std::string, std::string>& b) {
      return a.first < b.first;
  });

  if (file.is_open()) {

    // std::vector<BT::StringView> string_views = blackboard_->getKeys();
    // for (const auto & string_view : string_views) {
    //   file << string_view.data() << ":" << blackboard_->get<std::string>(string_view.data()) << std::endl;
    // }

    for (const auto & kv_pair : kv_pairs) {
      file << kv_pair.first << ":" << kv_pair.second << std::endl;
    }

    file.close();
    RCLCPP_INFO(get_logger(), "blackboard dumped to file: %s", filename.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "blackboard could NOT be dumped to file: %s", filename.c_str());
  }
}

}  // namespace BF
