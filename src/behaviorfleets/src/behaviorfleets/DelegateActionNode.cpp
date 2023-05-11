#include <string>
#include <iostream>


#include "behaviorfleets/DelegateActionNode.hpp"

#include "behaviortree_cpp/behavior_tree.h"
#include "bf_msgs/msg/mission_status.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "behaviortree_cpp/utils/shared_library.h"
#include "rclcpp/rclcpp.hpp"

#include "bf_msgs/msg/mission_command.hpp"
#include "bf_msgs/msg/mission_status.hpp"
#include <fstream>

namespace BF
{

DelegateActionNode::DelegateActionNode(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  std::string pkgpath, xml_path;
  
  remote_id_ = "not_set";
  remote_tree_ = "not_set";
  config().blackboard->get("node", node_);
  // config().blackboard->get("remote_tree", remote_tree_);
  // config().blackboard->get("remote_id", remote_id_);
  config().blackboard->get("pkgpath", pkgpath);
  
  getInput("remote_id", remote_id_);
  getInput("remote_tree", remote_tree_);

  std::cout << "remote_id: " << remote_id_ << std::endl;
  std::cout << "remote_tree: " << remote_tree_ << std::endl;
  
  xml_path = pkgpath + remote_tree_;
  std::cout << "xml_path: " << xml_path << std::endl;
  std::ifstream file(xml_path);
  std::ostringstream contents_stream;
  contents_stream << file.rdbuf();
  remote_tree_ = contents_stream.str();
  
  tree_pub_ = node_->create_publisher<bf_msgs::msg::MissionCommand>(
    "/" + remote_id_ + "/mission_command", 100);

  remote_sub_ = node_->create_subscription<bf_msgs::msg::MissionStatus>(
    "/" + remote_id_ + "/mission_status", rclcpp::SensorDataQoS(),
    std::bind(&DelegateActionNode::remote_status_callback, this, std::placeholders::_1));  
  
}

void
DelegateActionNode::remote_status_callback(bf_msgs::msg::MissionStatus::UniquePtr msg)
{
  remote_status_ = std::move(msg);
  remote_itentified_ = true;

  // This is useful for the *Any approach
  // if(!remote_itentified_){
  //   remote_id = msg->robot_id;
  //   remote_itentified_ = true;

  //   remote_sub_ = node_->create_subscription<bf_msgs::msg::MissionStatus>(
  //   "/" + remote_id + "/mission_status", rclcpp::SensorDataQoS(),
  //   std::bind(&DelegateActionNode::remote_status_callback, this, std::placeholders::_1));

  // }

  // std::cout << "remote status callback" << std::endl;

}

BT::NodeStatus
DelegateActionNode::tick()
{ 
  
  if(!remote_itentified_){
    bf_msgs::msg::MissionCommand msg;
    msg.mission_tree = remote_tree_;
    msg.robot_id = remote_id_;
    tree_pub_->publish(msg);
    std::cout << "tree publised in /" << remote_id_ << "/mission_command" << std::endl;
  } else {
    int status = remote_status_->status;
    switch(status) {
      case RUNNING:
        std::cout << "remote status: RUNNING" << std::endl;
        return BT::NodeStatus::RUNNING;
        break;
      case SUCCESS:
        std::cout << "remote status: SUCCESS" << std::endl;
        return BT::NodeStatus::SUCCESS;
        break;
      case FAILURE:
        std::cout << "remote status: FAILURE" << std::endl;
        return BT::NodeStatus::FAILURE;
        break;
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
