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
  std::chrono::milliseconds milis)
: Node(robot_id + "_blackboard_stresser"),
  robot_id_(robot_id)
{
  blackboard_ = BT::Blackboard::create();

  for (int i = 0; i < n_keys; i++) {
    keys_.push_back("key_" + std::to_string(i));
  }

  timer_ = create_wall_timer(milis, std::bind(&BlackboardStresser::control_cycle, this));
  
  rclcpp::on_shutdown([this]() { update_blackboard(); });
}

void BlackboardStresser::control_cycle()
{
  int prob = random_int(0, 100);
  if (random_int(0, 100) < 50) {
    update_blackboard();
  }
}

void BlackboardStresser::dump_blackboard()
{
  std::string filename = robot_id_ + "_blackboard_stresser.txt";
  std::ofstream file(filename);
  if (file.is_open()) {
    for (const std::string& str : keys_) {
      int aux = blackboard_->get<int>(str);
      file << str + ":" << aux << std::endl;
    }
    file.close();
    RCLCPP_INFO(get_logger(), "blackboard dumped to file: %s", filename.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "blackboard could NOT be dumped to file: %s", filename.c_str());
  }
}

void BlackboardStresser::update_blackboard()
{
  RCLCPP_INFO(get_logger(), "updating blackboard");
  int i = random_int(0, keys_.size() - 1);
  blackboard_->set(keys_[i], i);
}



int BlackboardStresser::random_int(int min, int max)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(min, max);

  return dis(gen);
}

}  // namespace BF
