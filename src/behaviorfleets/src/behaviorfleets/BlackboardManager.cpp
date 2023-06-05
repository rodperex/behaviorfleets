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

BlackboardManager::BlackboardManager(),
: Node("blackboard_manager")
{
    lock_ = false;
    robot_id_ = "";
    
    bb_pub_ = create_publisher<bf_msgs::msg::Blackboard>(
    "/shared_bb", 100);

    bb_sub_ = create_subscription<bf_msgs::msg::Blackboard>(
        "/shared_bb", rclcpp::SensorDataQoS(),
        std::bind(&BlackboardManager::blackboard_callback, this, std::placeholders::_1));
    
    blackboard_ = BT::Blackboard::create();

}

void control_cycle()
{
    if (!lock_ && !q_.empty()) {
        robot_id_  = q_.pop();
    }
}

void grant_bb() {
    bf_msgs::msg::Blackboard answ;
    answ.type = bf_msgs::Blackboard::GRANT;
    answ.robot_id = robot_id_;
    bb_pub_->publish(answ);
    lock_ = true;
}

void BlackboardManager::blackboard_callback(bf_msgs::msg::Blackboard msg)
{
    // bb_msg_ = std::move(msg);
    bf_msgs::msg::Blackboard answ;

    if (lock_) {
        if ((robot_id_ != msg->robot_id) && (msg->type == bf_msgs::Blackboard::REQUEST)) {
            q_.push(msg->robot_id);
        } else { // this robot was previously granted
            if (msg->type == bf_msgs::Blackboard::UPDATE) {
                // update the blackboard
            }
        }
        return;
    } else {
        if (msg->type == bf_msgs::Blackboard::REQUEST) {
            robot_id_ = msg->robot_id;
            grant_bb()
        }
    }
}

}  // namespace BF
