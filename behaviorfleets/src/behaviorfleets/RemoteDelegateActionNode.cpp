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

#include "behaviorfleets/RemoteDelegateActionNode.hpp"
#include <algorithm>
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

  RCLCPP_INFO(get_logger(), ("[ " + id_ + " ] " + "subscribed to /mission_poll").c_str());


  std::string ns = get_namespace();
  if (ns == "/") {
    mission_sub_ = create_subscription<bf_msgs::msg::Mission>(
      "/" + id_ + "/mission_command", rclcpp::SensorDataQoS(),
      std::bind(&RemoteDelegateActionNode::mission_callback, this, std::placeholders::_1));

    status_pub_ = create_publisher<bf_msgs::msg::Mission>(
      "/" + id_ + "/mission_status", 100);

  } else {  // a namespace has been set
    ns.erase(std::remove(ns.begin(), ns.end(), '/'), ns.end());
    id_ = ns;
    mission_sub_ = create_subscription<bf_msgs::msg::Mission>(
      "mission_command", rclcpp::SensorDataQoS(),
      std::bind(&RemoteDelegateActionNode::mission_callback, this, std::placeholders::_1));

    status_pub_ = create_publisher<bf_msgs::msg::Mission>(
      "mission_status", 100);
  }


  RCLCPP_INFO(
    get_logger(), ("[ " + id_ + " ] " + "subscribed to /" + id_ + "/mission_command").c_str());

  poll_pub_ = create_publisher<bf_msgs::msg::Mission>(
    "/mission_poll", 100);

  timer_ = create_wall_timer(50ms, std::bind(&RemoteDelegateActionNode::control_cycle, this));

  // plugins can be read from a topic as well
  this->declare_parameter("plugins", std::vector<std::string>());

  // new
  node_ = rclcpp::Node::make_shared("bt_node");
}


void
RemoteDelegateActionNode::control_cycle()
{
  bf_msgs::msg::Mission status_msg;

  status_msg.msg_type = bf_msgs::msg::Mission::STATUS;
  status_msg.robot_id = id_;
  status_msg.mission_id = mission_id_;
  status_msg.status = bf_msgs::msg::Mission::RUNNING;

  // in case the node has drained its requests trials, wait waiting_time_ seconds (randomized)
  auto elapsed = rclcpp::Clock().now() - t_last_request_;
  if ((elapsed.seconds() > waiting_time_) && (waiting_time_ > 0.0)) {
    n_tries_ = 0;
    waiting_time_ = 0.0;
    std::cout << "waiting time elapsed" << std::endl;
  }

  if (working_) {
    BT::NodeStatus status = tree_.rootNode()->executeTick();

    // spin bb_handler_ activate the callbacks to keep the shared blackboard updated
    rclcpp::spin_some(bb_handler_);
    // RCLCPP_INFO(get_logger(), "blackboard handler spinned");

    switch (status) {
      case BT::NodeStatus::RUNNING:
        status_msg.status = bf_msgs::msg::Mission::RUNNING;
        RCLCPP_DEBUG(get_logger(), ("[ " + id_ + " ] " + "RUNNING").c_str());
        break;
      case BT::NodeStatus::SUCCESS:
        status_msg.status = bf_msgs::msg::Mission::SUCCESS;
        RCLCPP_INFO(get_logger(), ("[ " + id_ + " ] " + "SUCCESS").c_str());
        working_ = false;
        break;
      case BT::NodeStatus::FAILURE:
        status_msg.status = bf_msgs::msg::Mission::FAILURE;
        RCLCPP_INFO(get_logger(), ("[ " + id_ + " ] " + "FAILURE").c_str());
        working_ = false;
        break;
    }
    status_pub_->publish(status_msg);
  } else {
    status_msg.status = bf_msgs::msg::Mission::IDLE;
  }
}

bool
RemoteDelegateActionNode::create_tree()
{
  BT::SharedLibrary loader;
  BT::BehaviorTreeFactory factory;

  std::vector<std::string> plugins = mission_->plugins;

  bool load_plugins = true;
  if (plugins.size() == 0) {
    load_plugins = false;
    // plugins = this->get_parameter("plugins").as_string_array();
    // RCLCPP_INFO(get_logger(), ("[ " + id_ + " ] " + "plugins not in the mission command").c_str());
    // if (plugins[0] == "none") {
    //   RCLCPP_INFO(get_logger(), ("[ " + id_ + " ] " + "no plugins to load").c_str());
    //   load_plugins = false;
    // }
  }

  try {
    if (load_plugins) {
      for (auto plugin : plugins) {
        factory.registerFromPlugin(loader.getOSName(plugin));
        RCLCPP_INFO(get_logger(), "plugin %s  ", plugin.c_str());
      }
    }

    auto blackboard = BT::Blackboard::create();
    // blackboard->set("node", shared_from_this());
    blackboard->set("node", node_);


    // create a blackboard handler to work with a shared blackboard
    bb_handler_ = std::make_shared<BlackboardHandler>(id_, blackboard);
    RCLCPP_INFO(get_logger(), "blackboard handler created");

    // execution CANNOT continue till the handler is synchronized with the global bb
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(3)) {
      rclcpp::spin_some(bb_handler_);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    tree_ = factory.createTreeFromText(mission_->mission_tree, blackboard);
    RCLCPP_INFO(get_logger(), "tree created");

    return true;
  } catch (std::exception & e) {
    bf_msgs::msg::Mission status_msg;
    status_msg.msg_type = bf_msgs::msg::Mission::STATUS;
    status_msg.robot_id = id_;
    status_msg.status = bf_msgs::msg::Mission::IDLE;
    status_pub_->publish(status_msg);
    RCLCPP_ERROR(get_logger(), ("[ " + id_ + " ] " + "ERROR creating tree: " + e.what()).c_str());
    return false;
  }
}

void
RemoteDelegateActionNode::mission_poll_callback(bf_msgs::msg::Mission::UniquePtr msg)
{
  if (msg->msg_type != bf_msgs::msg::Mission::OFFER) {
    // check if a REJECT message was received; if so, refrain from sending requests for a while
    // PROBLEM: the node will not respond to a new mission request until the timer expires
    // if (msg->msg_type != bf_msgs::msg::Mission::REJECT) {
    //   return;
    // }
    return;
  }
  // ignore missions if already working
  if (!working_) {
    mission_ = std::move(msg);

    if (((mission_->robot_id).length() > 0) && ((mission_->robot_id).compare(id_) != 0)) {
      RCLCPP_INFO(
        get_logger(),
        ("[ " + id_ + " ] " + "MISSION ignored: I am not " + mission_->robot_id).c_str());
      return;
    }
    if (((mission_->mission_id).compare(mission_id_) == 0) &&
      (n_tries_ < (MAX_REQUEST_TRIES_ - 1)))
    {
      bf_msgs::msg::Mission poll_msg;
      poll_msg.msg_type = bf_msgs::msg::Mission::REQUEST;
      poll_msg.robot_id = id_;
      poll_msg.mission_id = mission_id_;
      poll_msg.status = bf_msgs::msg::Mission::IDLE;
      poll_pub_->publish(poll_msg);
      n_tries_++;
      t_last_request_ = rclcpp::Clock().now();
      RCLCPP_INFO(
        get_logger(),
        ("[ " + id_ + " ] " + "REQUEST sent (" + std::to_string(n_tries_) +
        "): " + mission_id_).c_str());
    } else {  // either the mission is not for the node or the node is silent for a while
      if ((n_tries_ >= (MAX_REQUEST_TRIES_ - 1)) && (waiting_time_ == 0)) {
        // wait a random time (maximum MAX_WAITING_TIME_) before trying again
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.5, MAX_WAITING_TIME_);
        waiting_time_ = dis(gen);
        RCLCPP_INFO(
          get_logger(), ("[ " + id_ + " ] " + "waiting " + std::to_string(waiting_time_) +
          " seconds before trying again").c_str());
      }
      RCLCPP_INFO(get_logger(), ("[ " + id_ + " ] " + "unable to execute action").c_str());
    }
  } else {
    RCLCPP_INFO(get_logger(), ("[ " + id_ + " ] " + "MISSION ignored: busy").c_str());
  }
}

void
RemoteDelegateActionNode::mission_callback(bf_msgs::msg::Mission::UniquePtr msg)
{
  RCLCPP_INFO(get_logger(), ("[ " + id_ + " ] " + "mission callback").c_str());
  if (msg->msg_type != bf_msgs::msg::Mission::COMMAND) {
    return;
  }

  if (msg->msg_type == bf_msgs::msg::Mission::HALT) {
    RCLCPP_INFO(get_logger(), ("[ " + id_ + " ] " + "halt signal received").c_str());
    bf_msgs::msg::Mission status_msg;
    status_msg.msg_type = bf_msgs::msg::Mission::STATUS;
    status_msg.robot_id = id_;
    status_msg.status = bf_msgs::msg::Mission::IDLE;
    status_pub_->publish(status_msg);
    working_ = false;
  }

  // ignore missions if already working
  if (!working_) {
    RCLCPP_INFO(get_logger(), ("[ " + id_ + " ] " + "MISSION received").c_str());
    mission_ = std::move(msg);
    if (mission_->robot_id == id_) {
      RCLCPP_DEBUG(
        get_logger(), ("[ " + id_ + " ]\n" + mission_->mission_tree).c_str());
      working_ = create_tree();
    } else {
      RCLCPP_INFO(get_logger(), ("[ " + id_ + " ] " + "MISSION received, but not for me").c_str());
    }
  } else {
    RCLCPP_INFO(get_logger(), ("[ " + id_ + " ] " + "MISSION received, but I'm busy").c_str());
  }
}

void
RemoteDelegateActionNode::setID(std::string id)
{
  id_ = id;
}

}  // namespace BF
