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

#include <string>
#include <iostream>

#include "behaviortree_cpp/behavior_tree.h"
#include "bf_msgs/msg/mission_status.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "behaviortree_cpp/utils/shared_library.h"
#include "rclcpp/rclcpp.hpp"

#include "behaviorfleets/RemoteDelegateActionNodeAny.hpp"


namespace BF
{

RemoteDelegateActionNodeAny::RemoteDelegateActionNodeAny()
: Node("RemoteDelegateAN"),
id_("remote"),
mission_id_("generic")
{
  init();
}

RemoteDelegateActionNodeAny::RemoteDelegateActionNodeAny(const std::string robot_id,
const std::string mission_id)
: Node("RemoteDelegateAN"),
id_(robot_id),
mission_id_(mission_id)
{
  init();
}


void
RemoteDelegateActionNodeAny::init(){
  using namespace std::chrono_literals;

  poll_sub_ = create_subscription<bf_msgs::msg::MissionCommand>(
    "/mission_command", rclcpp::SensorDataQoS(),
    std::bind(&RemoteDelegateActionNodeAny::mission_poll_callback, this, std::placeholders::_1));

  mission_sub_ = create_subscription<bf_msgs::msg::MissionCommand>(
    "/" + id_ + "/mission_command", rclcpp::SensorDataQoS(),
    std::bind(&RemoteDelegateActionNodeAny::mission_callback, this, std::placeholders::_1));

  std::cout << "subscribed to " << "/" + id_ + "/mission_command"<< std::endl;

  poll_pub_ = create_publisher<bf_msgs::msg::MissionStatus>(
    "/mission_poll", 100);

  status_pub_ = create_publisher<bf_msgs::msg::MissionStatus>(
        "/" + id_ + "/mission_status", 100);

  timer_ = create_wall_timer(50ms, std::bind(&RemoteDelegateActionNodeAny::control_cycle, this));

  // plugins can be read from a topic as well
  this->declare_parameter("plugins", std::vector<std::string>());
}


void
RemoteDelegateActionNodeAny::control_cycle(){
  bf_msgs::msg::MissionStatus status_msg;

  status_msg.robot_id = id_;
  status_msg.status = RUNNING;

  if(working_) {
    BT::NodeStatus status = tree_.rootNode()->executeTick();
    switch(status) {
      case BT::NodeStatus::RUNNING:
        status_msg.status = RUNNING;
        std::cout << "RUNNING" << std::endl;
        break;
      case BT::NodeStatus::SUCCESS:
        status_msg.status = SUCCESS;
        std::cout << "SUCCESS" << std::endl;
        working_ = false;
        break;
      case BT::NodeStatus::FAILURE:
        status_msg.status = FAILURE;
        std::cout << "FAILURE" << std::endl;
        working_ = false;
        break;
    }
    status_pub_->publish(status_msg);
  } else {
    status_msg.status = IDLE;
  }
}

void
RemoteDelegateActionNodeAny::create_tree(){
  BT::SharedLibrary loader;
  BT::BehaviorTreeFactory factory;

  auto plugins = this->get_parameter("plugins").as_string_array();

  for(auto plugin : plugins) {
    factory.registerFromPlugin(loader.getOSName(plugin));
    std::cout << "plugin " << plugin << " loaded" << std::endl;
  }

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", shared_from_this());
  tree_ = factory.createTreeFromText(mission_->mission_tree, blackboard);

  std::cout << "tree created" << std::endl;
}

void
RemoteDelegateActionNodeAny::mission_poll_callback(bf_msgs::msg::MissionCommand::UniquePtr msg){
  // ignore missions if already working
  if(!working_) {
    mission_ = std::move(msg);
    if ((mission_->mission_id).compare(mission_id_) == 0){
      bf_msgs::msg::MissionStatus poll_msg;
      poll_msg.robot_id = id_;
      poll_msg.mission_id = mission_id_;
      poll_msg.status = IDLE;
      poll_pub_->publish(poll_msg);
      std::cout << "action request published: " << mission_id_ << std::endl;
    }else {
      std::cout << "unable to execute action: " << mission_id_ << std::endl;
    }
  }else {
    std::cout << "action request ignored" << std::endl;
  }
}

void
RemoteDelegateActionNodeAny::mission_callback(bf_msgs::msg::MissionCommand::UniquePtr msg){
  // ignore missions if already working
  if(!working_) {
    mission_ = std::move(msg);
    if (mission_->robot_id == id_){
      std::cout << "tree received" << std::endl << mission_->mission_tree << std::endl;
      create_tree();
      working_ = true;
    }else {
      std::cout << "tree received but not for this node" << std::endl;
    }
  }else {
    std::cout << "tree received but node is busy" << std::endl;
  }
}

void
RemoteDelegateActionNodeAny::setID(std::string id){
  id_ = id;
}

}  // namespace BF
