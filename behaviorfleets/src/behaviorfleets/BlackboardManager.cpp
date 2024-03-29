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
  using ::std::chrono_literals::operator""ms;
  msq_size_ = 10;
  init();
  RCLCPP_INFO(get_logger(), "control cycle: 50 ms");
  timer_cycle_ = create_wall_timer(50ms, std::bind(&BlackboardManager::control_cycle, this));
  copy_blackboard(blackboard);
}

BlackboardManager::BlackboardManager(
  BT::Blackboard::Ptr blackboard,
  std::chrono::milliseconds milis,
  int msq_size)
: Node("blackboard_manager"),
  msq_size_(msq_size)
{
  init();
  copy_blackboard(blackboard);

  RCLCPP_INFO(get_logger(), "control cycle: %ld ms", milis.count());
  timer_cycle_ = create_wall_timer(milis, std::bind(&BlackboardManager::control_cycle, this));
}

BlackboardManager::BlackboardManager(
  BT::Blackboard::Ptr blackboard, std::chrono::milliseconds milis,
  std::chrono::milliseconds bb_refresh_rate,
  int msq_size)
: Node("blackboard_manager"),
  msq_size_(msq_size)
{
  init();
  copy_blackboard(blackboard);

  RCLCPP_INFO(get_logger(), "control cycle: %ld ms", milis.count());
  timer_cycle_ = create_wall_timer(milis, std::bind(&BlackboardManager::control_cycle, this));
  RCLCPP_INFO(get_logger(), "blackboard refresh rate: %ld ms", bb_refresh_rate.count());
  timer_publish_ =
    create_wall_timer(bb_refresh_rate, std::bind(&BlackboardManager::publish_blackboard, this));
}
void BlackboardManager::init()
{
  lock_ = false;
  sync_ = false;
  robot_id_ = "";
  tam_q_ = 0;
  n_pub_ = 0;

  blackboard_ = BT::Blackboard::create();

  bb_pub_ = create_publisher<bf_msgs::msg::Blackboard>(
    "/blackboard", 100);

  bb_sub_ = create_subscription<bf_msgs::msg::Blackboard>(
    "/blackboard", rclcpp::SensorDataQoS().keep_last(msq_size_),
    std::bind(&BlackboardManager::blackboard_callback, this, std::placeholders::_1));

  // uncomment for testing
  // rclcpp::on_shutdown([this]() {dump_blackboard();});
}

void BlackboardManager::control_cycle()
{
  if (!lock_ && !q_.empty()) {
    if (q_.size() > tam_q_) {
      tam_q_ = q_.size();
      RCLCPP_INFO(get_logger(), "max.queue size: %zu", q_.size());
    }

    robot_id_ = q_.front();
    waiting_times_.push_back(rclcpp::Clock().now() - q_start_wait_.front());
    RCLCPP_INFO(
      get_logger(), "dequeuing robot %s (%zu pending). Waiting for %fs", q_.front().c_str(),
      q_.size() - 1,
      (rclcpp::Clock().now() - q_start_wait_.front()).nanoseconds() / 1e9);
    q_.pop();
    q_start_wait_.pop();
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
    q_start_wait_.push(rclcpp::Clock().now());
    RCLCPP_INFO(get_logger(), "request from robot %s enqueued", update_bb_msg_->robot_id.c_str());
  } else if ((update_bb_msg_->type == bf_msgs::msg::Blackboard::UPDATE) &&
    (update_bb_msg_->robot_id == robot_id_))
  {
    update_blackboard();  // attend request coming from the robot that has the blackboard
  } else if (update_bb_msg_->type == bf_msgs::msg::Blackboard::SYNC) {
    RCLCPP_INFO(
      get_logger(), "sychronization request received from %s", update_bb_msg_->robot_id.c_str());
    sync_ = true;
    publish_blackboard();
  }
}

void BlackboardManager::update_blackboard()
{
  RCLCPP_INFO(get_logger(), "%s updating blackboard", robot_id_.c_str());

  std::vector<std::string> keys = update_bb_msg_->keys;
  std::vector<std::string> values = update_bb_msg_->values;
  std::vector<std::string> types = update_bb_msg_->key_types;

  for (int i = 0; i < keys.size(); i++) {
    if (types[i] == "string" || types[i] == "unknown") {
      blackboard_->set(keys[i], values[i]);
    } else if (types[i] == "int") {
      blackboard_->set(keys[i], std::stoi(values[i]));
    } else if (types[i] == "float") {
      blackboard_->set(keys[i], std::stof(values[i]));
    } else if (types[i] == "double") {
      blackboard_->set(keys[i], std::stod(values[i]));
    } else if (types[i] == "bool") {
      blackboard_->set(keys[i], static_cast<bool>(std::stoi(values[i])));
    } else {
      RCLCPP_ERROR(get_logger(), "unknown type in the blackboard [%s]", types[i].c_str());
    }
  }

  lock_ = false;

  publish_blackboard();
}

void BlackboardManager::publish_blackboard()
{
  if (!lock_) {
    lock_ = true;
    bf_msgs::msg::Blackboard msg;
    std::vector<BT::StringView> string_views = blackboard_->getKeys();

    msg.type = bf_msgs::msg::Blackboard::PUBLISH;
    // msg.robot_id = "all";
    if (sync_) {
      RCLCPP_DEBUG(get_logger(), "responding to a synchronization request");
      msg.robot_id = "all";
      sync_ = false;
    } else {
      msg.robot_id = robot_id_;
    }

    std::vector<std::string> keys;
    std::vector<std::string> values;
    std::vector<std::string> types;

    for (const auto & string_view : string_views) {
      try {
        keys.push_back(string_view.data());
        values.push_back(blackboard_->get<std::string>(string_view.data()));
        types.push_back(get_type(string_view.data()));
        RCLCPP_DEBUG(
          get_logger(), "publishing key %s (%s)", string_view.data(),
          types.back().c_str());
      } catch (const std::exception & e) {
        RCLCPP_DEBUG(get_logger(), "key %s skipped", string_view.data());
      }
    }
    msg.keys = keys;
    msg.values = values;
    msg.key_types = types;
    bb_pub_->publish(msg);
    n_pub_++;
    lock_ = false;
    robot_id_ = "";

    RCLCPP_INFO(get_logger(), "blackboard published (%d)", n_pub_);
  }
}

void BlackboardManager::copy_blackboard(BT::Blackboard::Ptr source_bb)
{
  blackboard_->clear();

  std::vector<BT::StringView> string_views = source_bb->getKeys();
  for (const auto & string_view : string_views) {
    try {
      RCLCPP_DEBUG(get_logger(), "copying key %s", string_view.data());

      // check if the entry should be skipped
      if (string_view.find("efbb_") != std::string::npos) {
        RCLCPP_DEBUG(get_logger(), "key %s copy skipped", string_view.data());
        continue;
      }

      std::string type = get_type(source_bb, string_view.data());
      std::string val;
      if (type == "int") {
        int value = source_bb->get<int>(string_view.data());
        blackboard_->set(string_view.data(), value);
        val = std::to_string(value);
      } else if (type == "float") {
        float value = source_bb->get<float>(string_view.data());
        blackboard_->set(string_view.data(), value);
        val = std::to_string(value);
      } else if (type == "double") {
        double value = source_bb->get<double>(string_view.data());
        blackboard_->set(string_view.data(), value);
        val = std::to_string(value);
      } else if (type == "string") {
        std::string value = source_bb->get<std::string>(string_view.data());
        blackboard_->set(string_view.data(), value);
        val = value;
      } else if (type == "bool") {
        bool value = source_bb->get<bool>(string_view.data());
        blackboard_->set(string_view.data(), value);
        val = std::to_string(value);
      } else {
        throw std::runtime_error("unknown type");
        // RCLCPP_ERROR(get_logger(), "unknown type [%s]", type.c_str());
      }

      RCLCPP_DEBUG(get_logger(), "key %s copied", string_view.data());
      RCLCPP_DEBUG(get_logger(), "key %s value is: %s", string_view.data(), val.c_str());
      RCLCPP_DEBUG(
        get_logger(), "key %s type is: %s", string_view.data(),
        get_type(string_view.data()).c_str());
    } catch (const std::exception & e) {
      RCLCPP_DEBUG(get_logger(), "key %s copy skipped", string_view.data());
    }
  }
}

void BlackboardManager::dump_blackboard()
{
  RCLCPP_INFO(get_logger(), "dumping blackboard");
  std::string filename = "results/manager.txt";
  std::ofstream file(filename, std::ofstream::out);

  std::vector<std::pair<std::string, std::string>> kv_pairs;
  std::vector<BT::StringView> string_views = blackboard_->getKeys();

  for (const auto & string_view : string_views) {
    kv_pairs.push_back(
      std::make_pair(
        string_view.data(),
        blackboard_->get<std::string>(string_view.data())));
  }

  std::sort(
    kv_pairs.begin(), kv_pairs.end(),
    [](const std::pair<std::string, std::string> & a,
    const std::pair<std::string, std::string> & b) {
      return a.first < b.first;
    });

  if (file.is_open()) {
    for (const auto & kv_pair : kv_pairs) {
      file << kv_pair.first << ":" << kv_pair.second << std::endl;
    }

    file.close();
    RCLCPP_INFO(get_logger(), "blackboard dumped to file: %s", filename.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "blackboard could NOT be dumped to file: %s", filename.c_str());
  }

  dump_waiting_times();
}

void BlackboardManager::dump_waiting_times()
{
  RCLCPP_INFO(get_logger(), "dumping waiting times");
  std::string filename = "results/waiting_times.txt";
  std::ofstream file(filename, std::ofstream::out);

  if (file.is_open()) {
    for (const auto & wt : waiting_times_) {
      file << (wt.nanoseconds() / 1e6) << std::endl;
    }

    // last lines of the file is the maximum size of the queue and the number of bb pubs
    file << tam_q_ << std::endl;
    file << n_pub_ << std::endl;

    file.close();
    RCLCPP_INFO(get_logger(), "waiting times dumped to file: %s", filename.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "waiting times could NOT be dumped to file: %s", filename.c_str());
  }
}

std::string BlackboardManager::get_type(BT::Blackboard::Ptr bb, const char * port_name)
{

  const BT::PortInfo * port = bb->portInfo(port_name);
  // thirdparty library updated, so this is not working anymore. Not tested
  // const BT::TypeInfo * port = bb->entryInfo(port_name);

  int status;
  char * port_type = abi::__cxa_demangle(port->type().name(), nullptr, nullptr, &status);
  std::string type(port_type);
  std::free(port_type);

  if (type.find("Any") != std::string::npos) {
    return "string";
  }
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

std::string BlackboardManager::get_type(const char * port_name)
{

  const BT::PortInfo * port = blackboard_->portInfo(port_name);
  // thirdparty library updated, so this is not working anymore. Not tested
  // const BT::TypeInfo * port = blackboard_->entryInfo(port_name);
  int status;
  char * port_type = abi::__cxa_demangle(port->type().name(), nullptr, nullptr, &status);
  std::string type(port_type);
  std::free(port_type);

  if (type.find("Any") != std::string::npos) {
    return "string";
  }
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

}  // namespace BF
