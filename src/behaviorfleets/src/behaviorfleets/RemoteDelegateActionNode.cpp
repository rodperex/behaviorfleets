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

#include "behaviorfleets/RemoteDelegateActionNode.hpp"


namespace BF
{

RemoteDelegateActionNode::RemoteDelegateActionNode()
: Node("remote_delegate_action_node"),
  id_("remote"),
  mission_id_("generic")
{
  init();
}

RemoteDelegateActionNode::RemoteDelegateActionNode(
  const std::string robot_id,
  const std::string mission_id)
: Node(robot_id + "_remote_delegate_action_node"),
  id_(robot_id),
  mission_id_(mission_id)
{
  init();
}

void
RemoteDelegateActionNode::init()
{
  using namespace std::chrono_literals;

  poll_sub_ = create_subscription<bf_msgs::msg::Mission>(
    "/mission_poll", rclcpp::SensorDataQoS(),
    std::bind(&RemoteDelegateActionNode::mission_poll_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), ("[ " + id_ +" ] " + "subscribed to /mission_poll").c_str());

  mission_sub_ = create_subscription<bf_msgs::msg::Mission>(
    "/" + id_ + "/mission_command", rclcpp::SensorDataQoS(),
    std::bind(&RemoteDelegateActionNode::mission_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), ("[ " + id_ +" ] " + "subscribed to /"+ id_ +"/mission_command").c_str());

  poll_pub_ = create_publisher<bf_msgs::msg::Mission>(
    "/mission_poll", 100);

  status_pub_ = create_publisher<bf_msgs::msg::Mission>(
    "/" + id_ + "/mission_status", 100);

  timer_ = create_wall_timer(50ms, std::bind(&RemoteDelegateActionNode::control_cycle, this));

  // plugins can be read from a topic as well
  this->declare_parameter("plugins", std::vector<std::string>());
}


void
RemoteDelegateActionNode::control_cycle()
{
  bf_msgs::msg::Mission status_msg;

  status_msg.msg_type = bf_msgs::msg::Mission::STATUS;
  status_msg.robot_id = id_;
  status_msg.status = bf_msgs::msg::Mission::RUNNING;

  if (working_) {
    BT::NodeStatus status = tree_.rootNode()->executeTick();
    switch (status) {
      case BT::NodeStatus::RUNNING:
        status_msg.status = bf_msgs::msg::Mission::RUNNING;
        RCLCPP_INFO(get_logger(), ("[ " + id_ +" ] " + "RUNNING").c_str());
        break;
      case BT::NodeStatus::SUCCESS:
        status_msg.status = bf_msgs::msg::Mission::SUCCESS;
        RCLCPP_INFO(get_logger(), ("[ " + id_ +" ] " + "SUCCESS").c_str());
        working_ = false;
        break;
      case BT::NodeStatus::FAILURE:
        status_msg.status = bf_msgs::msg::Mission::FAILURE;
        RCLCPP_INFO(get_logger(), ("[ " + id_ +" ] " + "FAILURE").c_str());
        working_ = false;
        break;
    }
    status_pub_->publish(status_msg);
  } else {
    // if (can_do_it_) {
      status_msg.status = bf_msgs::msg::Mission::IDLE;
    // } else {
    //   status_msg.status = bf_msgs::msg::Mission::FAILURE;
    //   status_pub_->publish(status_msg);
    // }
  }
}

bool
RemoteDelegateActionNode::create_tree()
{
  BT::SharedLibrary loader;
  BT::BehaviorTreeFactory factory;

  std::vector<std::string> plugins = mission_->plugins;

  bool load_plugins = true;
  if (plugins.size() == 0)
  {
      plugins = this->get_parameter("plugins").as_string_array();
      RCLCPP_INFO(get_logger(), ("[ " + id_ +" ] " + "plugins not in the mission command").c_str());
    if (plugins[0] == "none") {
      RCLCPP_INFO(get_logger(), ("[ " + id_ +" ] " + "no plugins to load").c_str());
      load_plugins = false;
    }
  }

  try {
    if (load_plugins) {
      for (auto plugin : plugins) {
        factory.registerFromPlugin(loader.getOSName(plugin));
        RCLCPP_INFO(get_logger(), "plugin %s loaded", plugin.c_str());
      }
    }

    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", shared_from_this());
    tree_ = factory.createTreeFromText(mission_->mission_tree, blackboard);

    RCLCPP_INFO(get_logger(), "tree created");
    return true;
  } catch (std::exception & e) {
    bf_msgs::msg::Mission status_msg;
    status_msg.msg_type = bf_msgs::msg::Mission::STATUS;
    status_msg.robot_id = id_;
    status_msg.status = bf_msgs::msg::Mission::IDLE;
    status_pub_->publish(status_msg);
    RCLCPP_ERROR(get_logger(), ("[ " + id_ +" ] " + "ERROR creating tree: " + e.what()).c_str());
    return false;
  }
}

void
RemoteDelegateActionNode::mission_poll_callback(bf_msgs::msg::Mission::UniquePtr msg)
{
  if (msg->msg_type != bf_msgs::msg::Mission::COMMAND) {
    return;
  }
  // ignore missions if already working
  if (!working_) {
    // can_do_it_ = true;
    mission_ = std::move(msg);

    RCLCPP_INFO(get_logger(), ("[ " + id_ +" ] " + "robot_id to execute the bt: " + mission_->robot_id).c_str());
    if (((mission_->robot_id).length() > 0) && ((mission_->robot_id).compare(id_) != 0)) {
      RCLCPP_INFO(get_logger(), ("[ " + id_ +" ] " + "action request ignored: not for me").c_str());
      return;
    }
    if ((mission_->mission_id).compare(mission_id_) == 0) {
      bf_msgs::msg::Mission poll_msg;
      poll_msg.msg_type = bf_msgs::msg::Mission::REQUEST;
      poll_msg.robot_id = id_;
      poll_msg.mission_id = mission_id_;
      poll_msg.status = bf_msgs::msg::Mission::IDLE;
      poll_pub_->publish(poll_msg);
      RCLCPP_INFO(
        get_logger(),
        ("[ " + id_ +" ] " + "action request published:" + mission_id_).c_str());
    } else {
      RCLCPP_INFO(get_logger(), ("[ " + id_ +" ] " + "unable to execute action: " + mission_id_).c_str());
    }
  } else {
    RCLCPP_INFO(get_logger(), ("[ " + id_ +" ] " + "action request ignored: busy").c_str());
  }
}

void
RemoteDelegateActionNode::mission_callback(bf_msgs::msg::Mission::UniquePtr msg)
{
  if (msg->msg_type != bf_msgs::msg::Mission::COMMAND) {
    return;
  }

  if (msg->msg_type == bf_msgs::msg::Mission::HALT) {
    RCLCPP_INFO(get_logger(), ("[ " + id_ +" ] " + "halt signal received").c_str());
    bf_msgs::msg::Mission status_msg;
    status_msg.msg_type = bf_msgs::msg::Mission::STATUS;
    status_msg.robot_id = id_;
    status_msg.status = bf_msgs::msg::Mission::IDLE;
    status_pub_->publish(status_msg);
    working_ = false;
  }

  // ignore missions if already working
  if (!working_) {
    RCLCPP_INFO(get_logger(), ("[ " + id_ +" ] " + "mission received").c_str());
    mission_ = std::move(msg);
    if (mission_->robot_id == id_) {
      RCLCPP_INFO(get_logger(), ("[ " + id_ +" ] " + "tree received:\n" + mission_->mission_tree).c_str());
      working_ = create_tree();
      // can_do_it_ = working_;
    } else {
      RCLCPP_INFO(get_logger(), ("[ " + id_ +" ] " + "tree received but not for me").c_str());
    }
  } else {
    RCLCPP_INFO(get_logger(), ("[ " + id_ +" ] " + "tree received but I am busy").c_str());
  }
}

void
RemoteDelegateActionNode::setID(std::string id)
{
  id_ = id;
}

}  // namespace BF
