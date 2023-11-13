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
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  id_("remote"),
  mission_id_("generic")  
{
  init();
}

RemoteDelegateActionNode::RemoteDelegateActionNode(
  const std::string robot_id,
  const std::string mission_id)
: Node(robot_id + "_remote_delegate_action_node"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
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
  // this->declare_parameter("plugins", std::vector<std::string>());

  // new
  node_ = rclcpp::Node::make_shared("bt_node");

  // disconnection simulation
  tdisc_ = create_wall_timer(1ms, std::bind(&RemoteDelegateActionNode::sim_connectivity, this));
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
  if ((!working_) && (elapsed.seconds() > waiting_time_) && (waiting_time_ > 0.0)) {
    n_tries_ = 0;
    waiting_time_ = 0.0;
    RCLCPP_DEBUG(get_logger(), ("[ " + id_ + " ] " + "waiting time elapsed").c_str());
  }

  if (working_) {
    // if nobody is waiting for the mission status, do not publish it and stop working
    if (status_pub_->get_subscription_count() == 0) {
      RCLCPP_INFO(
        get_logger(),
        ("[ " + id_ + " ] " + "nobody is waiting for the status, STOPPING tree (?)").c_str());
      // working_ = false;
      // bb_handler_.reset();
      // return;
    }


    BT::NodeStatus status = tree_.rootNode()->executeTick();

    // spin bb_handler_ activate the callbacks to keep the shared blackboard updated
    if (!disconnected_) {
      rclcpp::spin_some(bb_handler_);
    }
    RCLCPP_DEBUG(get_logger(), "blackboard handler spinned");

    auto start_time = std::chrono::high_resolution_clock::now();
    auto end_time = start_time + std::chrono::seconds(5);
    switch (status) {
      case BT::NodeStatus::RUNNING:
        status_msg.status = bf_msgs::msg::Mission::RUNNING;
        RCLCPP_DEBUG(get_logger(), ("[ " + id_ + " ] " + "RUNNING").c_str());
        break;
      case BT::NodeStatus::SUCCESS:
        status_msg.status = bf_msgs::msg::Mission::SUCCESS;
        RCLCPP_INFO(get_logger(), ("[ " + id_ + " ] " + "***** SUCCESS *****").c_str());

        // IMPROVE THIS
        // the bb_handler needs to be spinned for a while in case there are pending updates to the global bb

        // while (bb_handler_->updating_bb()) {
        //   rclcpp::spin_some(bb_handler_);
        //   std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // }
        while (std::chrono::high_resolution_clock::now() < end_time) {
          rclcpp::spin_some(bb_handler_);
        }
        working_ = false;
        bb_handler_.reset();
        break;
      case BT::NodeStatus::FAILURE:
        status_msg.status = bf_msgs::msg::Mission::FAILURE;
        RCLCPP_INFO(get_logger(), ("[ " + id_ + " ] " + "FAILURE").c_str());
        while (std::chrono::high_resolution_clock::now() < end_time) {
          rclcpp::spin_some(bb_handler_);
        }
        working_ = false;
        bb_handler_.reset();
        break;
    }
    status_msg.source_id = mission_->source_id;
    status_pub_->publish(status_msg);
  } else {
    if (bb_handler_.get() != nullptr) {
      bb_handler_.reset();
      RCLCPP_DEBUG(get_logger(), ("[ " + id_ + " ] " + "bb_handler reset").c_str());
    }
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
    RCLCPP_INFO(get_logger(), ("[ " + id_ + " ] " + "plugins not in the mission command").c_str());
    // if (plugins[0] == "none") {
    //   RCLCPP_INFO(get_logger(), ("[ " + id_ + " ] " + "no plugins to load").c_str());
    //   load_plugins = false;
    // }
  }

  try {
    if (load_plugins) {
      for (auto plugin : plugins) {
        factory.registerFromPlugin(loader.getOSName(plugin));
        RCLCPP_DEBUG(get_logger(), "plugin %s  ", plugin.c_str());
      }
    }

    auto blackboard = BT::Blackboard::create();
    // blackboard->set("node", shared_from_this())
    blackboard->set("node", node_);
    // insert the name of the robot in case it is useful (excluded from sharing)
    blackboard->set("efbb_robot_id", id_);
    RCLCPP_DEBUG(get_logger(), "blackboard created + robot_id (%s) & node inserted", id_.c_str());


    // create a blackboard handler to work with a shared blackboard
    bb_handler_ = std::make_shared<BlackboardHandler>(id_ + "_bbh", blackboard);
    RCLCPP_DEBUG(get_logger(), "blackboard handler created");

    // execution CANNOT continue till the handler is synchronized with the global bb
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(3)) {
      rclcpp::spin_some(bb_handler_);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    tree_ = factory.createTreeFromText(mission_->mission_tree, blackboard);
    RCLCPP_INFO(get_logger(), "MISSION TREE created. Robot WORKING...");

    return true;
  } catch (std::exception & e) {
    bf_msgs::msg::Mission status_msg;
    status_msg.msg_type = bf_msgs::msg::Mission::STATUS;
    status_msg.robot_id = id_;
    status_msg.status = bf_msgs::msg::Mission::IDLE;
    status_msg.source_id = mission_->source_id;
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
        ("[ " + id_ + " ] " + "MISSION ignored (code " +
        std::to_string(bf_msgs::msg::Mission::OFFER) + "): I'm not " +
        mission_->robot_id).c_str());
      return;
    }

    if (((mission_->mission_id).compare(mission_id_) == 0) &&
      (n_tries_ < (MAX_REQUEST_TRIES_ - 1)))
    {
      bf_msgs::msg::Mission poll_msg;
      poll_msg.msg_type = bf_msgs::msg::Mission::REQUEST;
      poll_msg.robot_id = id_;
      poll_msg.mission_id = mission_id_;
      poll_msg.source_id = mission_->source_id; // NEW
      poll_msg.status = bf_msgs::msg::Mission::IDLE;
      poll_pub_->publish(poll_msg);
      n_tries_++;
      t_last_request_ = rclcpp::Clock().now();
      RCLCPP_INFO(
        get_logger(),
        ("[ " + id_ + " ] " + "REQUEST sent (" + std::to_string(n_tries_) +
        ") to " + mission_->source_id + ": " + mission_id_).c_str());
    } else {  // either the mission is not for the node or the node is silent for a while
      if ((n_tries_ >= (MAX_REQUEST_TRIES_ - 1)) && (waiting_time_ == 0)) {
        // wait a random time (maximum MAX_WAITING_TIME_) before trying again
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.5, MAX_WAITING_TIME_);
        waiting_time_ = dis(gen);
        RCLCPP_DEBUG(
          get_logger(), ("[ " + id_ + " ] " + "WAITING " + std::to_string(waiting_time_) +
          " seconds before trying again").c_str());
      }
      RCLCPP_DEBUG(get_logger(), ("[ " + id_ + " ] " + "unable to execute MISSION").c_str());
    }
  } else {
    RCLCPP_DEBUG(get_logger(), ("[ " + id_ + " ] " + "OFFER ignored, I'm BUSY").c_str());
  }
}

void
RemoteDelegateActionNode::mission_callback(bf_msgs::msg::Mission::UniquePtr msg)
{
  RCLCPP_DEBUG(get_logger(), ("[ " + id_ + " ] " + "mission callback").c_str());
  if ((msg->msg_type != bf_msgs::msg::Mission::COMMAND) &&
    msg->msg_type != bf_msgs::msg::Mission::HALT)
  {
    RCLCPP_INFO(
      get_logger(),
      ("[ " + id_ + " ] " + "Wrong message type received").c_str());
    return;
  }

  if (msg->msg_type == bf_msgs::msg::Mission::HALT) {
    RCLCPP_INFO(get_logger(), ("[ " + id_ + " ] " + "HALT signal received").c_str());
    bf_msgs::msg::Mission status_msg;
    status_msg.msg_type = bf_msgs::msg::Mission::STATUS;
    status_msg.robot_id = id_;
    status_msg.status = bf_msgs::msg::Mission::IDLE;
    status_msg.source_id = mission_->source_id;
    status_pub_->publish(status_msg);
    working_ = false;
    bb_handler_.reset();
  }

  // ignore missions if already working
  if (!working_) {
    RCLCPP_INFO(
      get_logger(), ("[ " + id_ + " ] " + "MISSION received (" + msg->source_id + ")").c_str());
    mission_ = std::move(msg);
    if (mission_->robot_id == id_) {
      RCLCPP_DEBUG(
        get_logger(), ("[ " + id_ + " ]\n" + mission_->mission_tree).c_str());
      working_ = create_tree();
    } else {
      RCLCPP_DEBUG(
        get_logger(),
        ("[ " + id_ + " ] " + "MISSION ignored (" + msg->source_id + "), not for me").c_str());
    }
  } else {
    RCLCPP_DEBUG(
      get_logger(),
      ("[ " + id_ + " ] " + "MISSION ignored (" + msg->source_id + "), I'm BUSY").c_str());
  }
}

void
RemoteDelegateActionNode::setID(std::string id)
{
  id_ = id;
}

// disconnection simulation
void RemoteDelegateActionNode::sim_connectivity() {
  try {
      std::cout << tf_buffer_.allFramesAsString() << std::endl;
      std::cout << "-------------------" << std::endl;
      geometry_msgs::msg::TransformStamped transform = tf_buffer_ .lookupTransform("map", "base_link", tf2::TimePoint());

      double x = transform.transform.translation.x;
      double y = transform.transform.translation.y;

      double dist = std::sqrt(std::pow(x_hotspot_ - x, 2) + std::pow(y_hotspot_ - y, 2));

      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_real_distribution<double> distribution(0.0, 1.0);

      disconnected_ = distribution(gen) > gaussian_probability(dist);
      
      RCLCPP_INFO(get_logger(), "Current robot pose: x=%.2f, y=%.2f", x, y);
      if (disconnected_) {
        RCLCPP_INFO(get_logger(), "Robot disconnected");
      }
  }
  catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(get_logger(), "Failed to lookup transform: %s", ex.what());
  }
}
double RemoteDelegateActionNode::gaussian_probability(double distance) {
  // Calculate the z-score
  double z = (distance - disc_mean_) / disc_stddev_;

  // Use the error function (erf) to get the cumulative probability
  return 0.5 * (1.0 + std::erf(z / std::sqrt(2.0)));
}

}  // namespace BF
