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

namespace BF
{

BlackboardHandler::BlackboardHandler(
  const std::string robot_id,
  BT::Blackboard::Ptr blackboard)
: Node(robot_id + "_blackboard_handler"),
  robot_id_(robot_id),
  blackboard_(blackboard),
  access_granted_(false),
  request_sent_(false),
  n_success_(0),
  n_requests_(0),
  n_updates_(0)
{
  using namespace std::chrono_literals;

  bb_cache_ = BT::Blackboard::create();
  cache_blackboard();

  bb_pub_ = create_publisher<bf_msgs::msg::Blackboard>(
    "/blackboard", 100);

  bb_sub_ = create_subscription<bf_msgs::msg::Blackboard>(
    "/blackboard", rclcpp::SensorDataQoS().keep_last(1000),
    std::bind(&BlackboardHandler::blackboard_callback, this, std::placeholders::_1));

  timer_ = create_wall_timer(10ms, std::bind(&BlackboardHandler::control_cycle, this));

  sync_rcvd_ = false;

  sync_bb();

  // rclcpp::on_shutdown([this]() {dump_data();});
}

BlackboardHandler::~BlackboardHandler()
{
  dump_data();
}

void BlackboardHandler::dump_data()
{
  std::string filename = "results/" + robot_id_ + ".txt";
  std::ofstream file(filename, std::ofstream::out);
  if (file.is_open()) {
    RCLCPP_DEBUG(get_logger(), "total waiting time = %f ms", (waiting_time_.nanoseconds() / 1e6));
    RCLCPP_DEBUG(get_logger(), "number of successful requests =  %d", n_success_);
    RCLCPP_DEBUG(get_logger(), "avg. waiting time =  %f ms", (waiting_time_.nanoseconds() / 1e6));
    file << "avg_wt:" << ((waiting_time_.nanoseconds() / 1e6) / n_success_) <<
      std::endl;
    file << "requests:" << n_requests_ << std::endl;
    file << "success:" << n_success_ << std::endl;
    file << "updates:" << n_updates_ << std::endl;
    file.close();
  }
}

void BlackboardHandler::control_cycle()
{
  if (has_bb_changed()) {
    update_blackboard();
    cache_blackboard();
  }
  // dump_data();
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
      // RCLCPP_INFO(get_logger(), "key %s not in cache", entry_bb.data());
      return true;
    } else if (blackboard_->get<std::string>(entry_bb.data()) !=
      bb_cache_->get<std::string>(entry_bb.data()))
    {
      // RCLCPP_INFO(get_logger(), "key %s has changed", entry_bb.data());
      return true;
    }
  }
  return false;
}

void BlackboardHandler::blackboard_callback(bf_msgs::msg::Blackboard::UniquePtr msg)
{
  RCLCPP_DEBUG(get_logger(), "blackboard_callback");
  if ((msg->type == bf_msgs::msg::Blackboard::GRANT) && (msg->robot_id == robot_id_)) {
    // RCLCPP_INFO(get_logger(), "access to blackboard granted");
    access_granted_ = true;
    update_blackboard();
    return;
  }
  if ((msg->type == bf_msgs::msg::Blackboard::PUBLISH) && (msg->robot_id == robot_id_)) {
    RCLCPP_INFO(get_logger(), "published global blackboard is mine");
    n_updates_++;
  }
  if ((msg->type == bf_msgs::msg::Blackboard::PUBLISH) && (msg->robot_id != robot_id_)) {
    sync_rcvd_ = true;
    RCLCPP_INFO(get_logger(), "updating local blackboard from the shared one");
    n_updates_++;
    for (int i = 0; i < msg->keys.size(); i++) {
      if (msg->key_types[i] == "string") {
        blackboard_->set(msg->keys.at(i), msg->values.at(i));
      } else if (msg->key_types[i] == "int") {
        blackboard_->set(msg->keys.at(i), std::stoi(msg->values.at(i)));
      } else if (msg->key_types[i] == "float") {
        blackboard_->set(msg->keys.at(i), std::stof(msg->values.at(i)));
      } else if (msg->key_types[i] == "double") {
        blackboard_->set(msg->keys.at(i), std::stod(msg->values.at(i)));
      } else if (msg->key_types[i] == "bool") {
        blackboard_->set(msg->keys.at(i), (bool)std::stoi(msg->values.at(i)));
      } else {
        RCLCPP_ERROR(get_logger(), "Unknown type [%s]", msg->key_types[i].c_str());
      }
    }
    cache_blackboard();
    return;
  }
  if ((msg->type == bf_msgs::msg::Blackboard::DENY) && (msg->robot_id != robot_id_)) {
    RCLCPP_INFO(get_logger(), "access to blackboard denied");
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

      //check if the entry should be skipped
      if (string_view.find("efbb_") != std::string::npos) {
        excluded_keys_.push_back(string_view.data());
        RCLCPP_INFO(get_logger(), "key %s excluded", string_view.data());
        continue;
      }
      std::string type = get_type(string_view.data());

      if (type == "string" || type == "unknown") {
        bb_cache_->set(string_view.data(), blackboard_->get<std::string>(string_view.data()));
      } else if (type == "int") {
        bb_cache_->set(string_view.data(), blackboard_->get<int>(string_view.data()));
      } else if (type == "float") {
        bb_cache_->set(string_view.data(), blackboard_->get<float>(string_view.data()));
      } else if (type == "double") {
        bb_cache_->set(string_view.data(), blackboard_->get<double>(string_view.data()));
      } else if (type == "bool") {
        bb_cache_->set(string_view.data(), blackboard_->get<bool>(string_view.data()));
      } else {
        RCLCPP_ERROR(get_logger(), "unknown type [%s]", type.c_str());
      }

      // RCLCPP_INFO(get_logger(), "key %s cached", string_view.data());
    } catch (const std::exception & e) {
      excluded_keys_.push_back(string_view.data());
      // RCLCPP_INFO(get_logger(), "key %s skipped", string_view.data());
    }
  }
}

void BlackboardHandler::update_blackboard()
{
  bf_msgs::msg::Blackboard msg;

  if (access_granted_) {
    waiting_time_ = (rclcpp::Clock().now() - t_last_request_) + waiting_time_;
    n_success_++;
    avg_waiting_time_ = (waiting_time_.nanoseconds() / n_success_) / 1e6; // millis
    RCLCPP_INFO(
      get_logger(), "SUCCESS %d: updating shared blackboard (%f ms)", n_success_,
      avg_waiting_time_);
    std::vector<BT::StringView> string_views = blackboard_->getKeys();
    msg.robot_id = robot_id_;
    msg.type = bf_msgs::msg::Blackboard::UPDATE;
    std::vector<std::string> keys;
    std::vector<std::string> values;
    std::vector<std::string> types;
    msg.values = {};
    for (const auto & string_view : string_views) {
      if (std::find(
          excluded_keys_.begin(), excluded_keys_.end(),
          string_view.data()) == excluded_keys_.end())
      {
        keys.push_back(string_view.data());
        values.push_back(blackboard_->get<std::string>(string_view.data()));
        types.push_back(get_type(string_view.data()));
      }
    }
    msg.keys = keys;
    msg.values = values;
    msg.key_types = types;
    bb_pub_->publish(msg);
    request_sent_ = false;
    access_granted_ = false;
  } else {
    // RCLCPP_INFO(get_logger(), "requesting access to blackboard");
    msg.type = bf_msgs::msg::Blackboard::REQUEST;
    msg.robot_id = robot_id_;
    if (!request_sent_) {
      bb_pub_->publish(msg);
      request_sent_ = true;
      n_requests_++;
      t_last_request_ = rclcpp::Clock().now();
    } else {
      // RCLCPP_INFO(get_logger(), "waiting for access to blackboard");
      if ((rclcpp::Clock().now() - t_last_request_).seconds() > 5.0) {
        RCLCPP_INFO(get_logger(), "request timed out");
        request_sent_ = false;
      }
    }
  }
}

void BlackboardHandler::sync_bb() {
  RCLCPP_INFO(get_logger(), "Synchronizing with global blackboard");

  bf_msgs::msg::Blackboard msg;
  msg.type = bf_msgs::msg::Blackboard::SYNC;
  msg.robot_id = robot_id_; 
  bb_pub_->publish(msg);
}

std::string BlackboardHandler::get_type(const char * port_name)
{
  const BT::PortInfo * port = blackboard_->portInfo(port_name);
  int status;
  char * port_type = abi::__cxa_demangle(port->type().name(), nullptr, nullptr, &status);
  std::string type(port_type);
  std::free(port_type);

  if (type.find("string") != std::string::npos) {
    return "string";
  }
  if (type.find("int") != std::string::npos) {
    return "int";
  }
  if (type.find("float") != std::string::npos) {
    return "float";
  }
  if (type.find("double") != std::string::npos) {
    return "double";
  }
  if (type.find("bool") != std::string::npos) {
    return "bool";
  }

  return "unknown";
}

} // namespace BF
