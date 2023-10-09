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

#include "test/BlackboardStresser.hpp"

namespace BF
{

BlackboardStresser::BlackboardStresser(
  const std::string robot_id,
  const int n_keys,
  std::chrono::milliseconds milis,
  std::chrono::seconds op_time,
  std::chrono::seconds delay
)
: Node(robot_id + "_blackboard_stresser"),
  robot_id_(robot_id),
  op_time_(op_time + delay),
  delay_(delay),
  n_changes_(0),
  bb_handler_spinning_(true)
{
  blackboard_ = BT::Blackboard::create();
  bb_handler_ = std::make_shared<BlackboardHandler>(robot_id_ + "_handler", blackboard_);

  // spin the handler in a separate thread
  spin_thread_ = std::thread(
    [this]() {
      // while (bb_handler_spinning_) {
      //   rclcpp::spin_some(bb_handler_);
      // }
      // bb_handler_->get_node_base_interface()->get_context()->shutdown("stress test finished");
      // RCLCPP_INFO(get_logger(), "blackboard handler %s finished", robot_id_.c_str());
      rclcpp::spin(bb_handler_);
    });
  spin_thread_.detach();

  for (int i = 0; i < n_keys; i++) {
    keys_.push_back("key_" + std::to_string(i));
    blackboard_->set(keys_[i], i);
  }

  RCLCPP_INFO(
    get_logger(), "blackboard stresser %s; period = %ld milis - delay = %ld seconds",
    robot_id_.c_str(), milis.count(), delay_.count());

  timer_ = create_wall_timer(milis, std::bind(&BlackboardStresser::control_cycle, this));

  // double durationInSeconds = 0.001; // 0.001 seconds
  // std::chrono::milliseconds durationMillis = std::chrono::duration_cast<std::chrono::milliseconds>(
  //     std::chrono::duration<double>(durationInSeconds)
  // );
  // timer2_ = create_wall_timer(durationMillis, std::bind(&BlackboardStresser::control_cycle2, this));

  t_start_ = rclcpp::Clock().now();

  // rclcpp::on_shutdown([this]() {dump_blackboard();});

  rclcpp::on_shutdown(
    [this]() {
      std::cout << "on_shutdown" << std::endl;
      // bb_handler_spinning_ = false;
      if (spin_thread_.joinable()) {
        spin_thread_.join();
      }
      std::cout << "spin_thread JOINED" << std::endl;
    });
}

BlackboardStresser::~BlackboardStresser()
{
  std::cout << "BlackboardStresser destructor" << std::endl;
  dump_blackboard();
}

void BlackboardStresser::control_cycle()
{
  if ((rclcpp::Clock().now() - t_start_ < delay_) && delay_ != std::chrono::seconds(0)) {
    return;
  }
  

  if (op_time_ == std::chrono::seconds(0) ||
    rclcpp::Clock().now() - t_start_ < op_time_)
  {
    // if (random_int(0, 100) < 50) {
    //   n_changes_++;
    //   update_blackboard();
    // }
    n_changes_++;
    update_blackboard();
  } else {
    // dump_blackboard();
    // bb_handler_->get_node_base_interface()->get_context()->shutdown("stress test finished");
  }

  // rclcpp::spin_some(bb_handler_);
}

void BlackboardStresser::control_cycle2()
{
  if ((rclcpp::Clock().now() - t_start_ < delay_) && delay_ != std::chrono::seconds(0)) {
    return;
  }
  rclcpp::spin_some(bb_handler_);
}

void BlackboardStresser::dump_blackboard()
{
  std::string filename = "results/" + robot_id_ + ".txt";
  std::ofstream file(filename, std::ofstream::out);
  if (file.is_open()) {
    for (const std::string & str : keys_) {
      file << str + ":" << blackboard_->get<std::string>(str) << std::endl;
    }
    file << "n_changes:" << n_changes_ << std::endl;
    file.close();
    // RCLCPP_INFO(get_logger(), "blackboard dumped to file: %s", filename.c_str());
  } else {
    // RCLCPP_INFO(get_logger(), "blackboard could NOT be dumped to file: %s", filename.c_str());
  }
}

void BlackboardStresser::update_blackboard()
{
  int i = random_int(0, keys_.size() - 1);
  int val = random_int(0, 100);
  RCLCPP_INFO(get_logger(), "updating key: %s to %d", keys_[i].c_str(), val);
  blackboard_->set(keys_[i], val);
}

int BlackboardStresser::random_int(int min, int max)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(min, max);

  return dis(gen);
}

}  // namespace BF
