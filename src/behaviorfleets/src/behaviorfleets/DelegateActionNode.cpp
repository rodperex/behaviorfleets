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
#include <fstream>
#include <sstream>

#include "behaviorfleets/DelegateActionNode.hpp"

#include "behaviortree_cpp/behavior_tree.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "behaviortree_cpp/utils/shared_library.h"
#include "rclcpp/rclcpp.hpp"

#include "bf_msgs/msg/mission.hpp"

namespace BF
{

DelegateActionNode::DelegateActionNode(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  std::string pkgpath, xml_path;

  remote_id_ = "";
  remote_tree_ = "not_set";
  timeout_ = -1.0;
  MAX_TRIES_ = -1;
  config().blackboard->get("node", node_);
  config().blackboard->get("pkgpath", pkgpath);

  getInput("remote_tree", remote_tree_);
  getInput("mission_id", mission_id_);
  getInput("remote_id", remote_id_);
  getInput("timeout", timeout_);
  getInput("max_tries", MAX_TRIES_);

  std::string plugins_str;
  getInput("plugins", plugins_str);
  decode(plugins_str, &plugins_);

  std::string exclude_str;
  getInput("exclude", exclude_str);
  decode(exclude_str, &excluded_);

  RCLCPP_INFO(node_->get_logger(), "plugins to propagate: %ld", plugins_.size());
  for (const auto & str : plugins_) {
    RCLCPP_INFO(node_->get_logger(), str.c_str());
  }

  RCLCPP_INFO(node_->get_logger(), "remote tree: %s", remote_tree_.c_str());
  RCLCPP_INFO(node_->get_logger(), "mission id: %s", mission_id_.c_str());

  xml_path = pkgpath + remote_tree_;
  RCLCPP_INFO(node_->get_logger(), "xml_path: %s", xml_path.c_str());
  std::ifstream file(xml_path);
  std::ostringstream contents_stream;
  contents_stream << file.rdbuf();
  remote_tree_ = contents_stream.str();

  mission_pub_ = node_->create_publisher<bf_msgs::msg::Mission>(
    "/mission_poll", 100);

  poll_sub_ = node_->create_subscription<bf_msgs::msg::Mission>(
    "/mission_poll", rclcpp::SensorDataQoS(),
    std::bind(&DelegateActionNode::mission_poll_callback, this, std::placeholders::_1));
}

void
DelegateActionNode::reset()
{
  RCLCPP_INFO(node_->get_logger(), "reset");
  remote_identified_ = false;
  getInput("remote_id", remote_id_);
  std::cout << "remote_id_: " << remote_id_ << std::endl;
  mission_pub_ = node_->create_publisher<bf_msgs::msg::Mission>(
    "/mission_poll", 100);
  remote_sub_.reset();
}

void
DelegateActionNode::decode(std::string str, std::vector<std::string> * vector)
{
  std::stringstream ss(str);
  std::string item;
  while (std::getline(ss, item, ',')) {
    // remove leading white spaces
    size_t start = item.find_first_not_of(" ");
    if (start != std::string::npos) {
      item = item.substr(start);
    }
    // remove trailing white spaces
    size_t end = item.find_last_not_of(" ");
    if (end != std::string::npos) {
      item = item.substr(0, end + 1);
    }

    vector->push_back(item);
  }
}

void
DelegateActionNode::remote_status_callback(bf_msgs::msg::Mission::UniquePtr msg)
{
  RCLCPP_INFO(
    node_->get_logger(), (std::string("remote status received: ") +
    "[ " + msg->robot_id + " : " + msg->mission_id + " ]").c_str());
  remote_status_ = std::move(msg);
  t_last_status_ = node_->now();
}

void
DelegateActionNode::mission_poll_callback(bf_msgs::msg::Mission::UniquePtr msg)
{
  if (msg->msg_type != bf_msgs::msg::Mission::REQUEST) {
    return;
  }
  RCLCPP_INFO(
    node_->get_logger(), (std::string("poll request received: ") +
    "[ " + msg->robot_id + " : " + msg->mission_id + " ]").c_str());
  // ignore answers from other robots
  if (!remote_identified_) {
    // check if the remote should be excluded
    if (is_remote_excluded(msg->robot_id)) {
      RCLCPP_INFO(
        node_->get_logger(), (std::string("remote excluded: ") +
        "[ " + msg->robot_id + " ]").c_str());
        // TO DO: publish a negative answer
        // bf_msgs::msg::Mission mission_msg;
        // mission_msg.msg_type = bf_msgs::msg::Mission::REJECT;
        // mission_msg.robot_id = msg->robot_id;
        // mission_msg.mission_id = msg->mission_id;
    }
    poll_answ_ = std::move(msg);
    remote_id_ = poll_answ_->robot_id;
    RCLCPP_INFO(
      node_->get_logger(), (std::string("remote identified: ") +
      "[ " + remote_id_ + " : " + poll_answ_->mission_id + " ]").c_str());

    remote_sub_ = node_->create_subscription<bf_msgs::msg::Mission>(
      "/" + remote_id_ + "/mission_status", rclcpp::SensorDataQoS(),
      std::bind(&DelegateActionNode::remote_status_callback, this, std::placeholders::_1));

    mission_pub_ = node_->create_publisher<bf_msgs::msg::Mission>(
      "/" + remote_id_ + "/mission_command", 100);

    bf_msgs::msg::Mission mission_msg;
    mission_msg.msg_type = bf_msgs::msg::Mission::COMMAND;
    mission_msg.robot_id = remote_id_;
    mission_msg.mission_tree = remote_tree_;
    mission_msg.plugins = plugins_;
    mission_pub_->publish(mission_msg);
    RCLCPP_INFO(node_->get_logger(), "tree publised");
    RCLCPP_INFO(node_->get_logger(), "status in /%s/mission_status", remote_id_.c_str());

    remote_identified_ = true;
    t_last_status_ = node_->now();
  }
}

bool
DelegateActionNode::is_remote_excluded(std::string remote_id)
{
  for (const std::string & str : excluded_) {
    if (str == remote_id) {
      return true;
    }
  }
  return false;
}

BT::NodeStatus
DelegateActionNode::tick()
{
  if (!remote_identified_) {
    bf_msgs::msg::Mission msg;
    msg.msg_type = bf_msgs::msg::Mission::COMMAND;
    msg.mission_id = mission_id_;
    msg.robot_id = remote_id_;
    mission_pub_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "mission %s publised", mission_id_.c_str());
  } else {
    if (remote_status_ != nullptr) {
      auto elapsed = node_->now() - t_last_status_;
      if ((elapsed.seconds() > timeout_) && (timeout_ != -1)) {
        RCLCPP_INFO(
          node_->get_logger(), (std::string("remote ") + "[ " + remote_id_ + " ] " +
          "timed out: looking for a new one").c_str());
        n_tries_++;
        reset();
        RCLCPP_INFO(node_->get_logger(), std::string("tries: " + std::to_string(n_tries_) +
          " / " + std::to_string(MAX_TRIES_)).c_str());
        if ((n_tries_ >= MAX_TRIES_) && (MAX_TRIES_ != -1))
        {
          RCLCPP_INFO(
            node_->get_logger(), (std::string("remote ") + "[ " + remote_id_ + " ] " +
            "timed out: max number of tries reached").c_str());
          n_tries_ = 0;
          return BT::NodeStatus::FAILURE;
        }
      } else {
        int status = remote_status_->status;
        switch (status) {
          case bf_msgs::msg::Mission::RUNNING:
            RCLCPP_INFO(
              node_->get_logger(), (std::string("remote status ") +
              "[ " + remote_id_ + " ]: " + "RUNNING").c_str());
            return BT::NodeStatus::RUNNING;
            break;
          case bf_msgs::msg::Mission::SUCCESS:
            RCLCPP_INFO(
              node_->get_logger(), (std::string("remote status ") +
              "[ " + remote_id_ + " ]: " + "SUCCESS").c_str());
            reset();
            return BT::NodeStatus::SUCCESS;
            break;
          case bf_msgs::msg::Mission::FAILURE:
            RCLCPP_INFO(
              node_->get_logger(), (std::string("remote status ") +
              "[ " + remote_id_ + " ]: " + "FAILURE").c_str());
            reset();
            return BT::NodeStatus::FAILURE;
            break;
          case bf_msgs::msg::Mission::IDLE:
            RCLCPP_INFO(
              node_->get_logger(), (std::string("remote status ") +
              "[ " + remote_id_ + " ]: " + "IDLE").c_str());
            reset();
            break;
        }
      }
    }
  }

  return BT::NodeStatus::RUNNING;
}

}  // namespace BF

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<BF::DelegateActionNode>("DelegateActionNode");
}
