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

namespace BF
{

DelegateActionNode::DelegateActionNode(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  std::string tree;
  std::string remote_id;

  config().blackboard->get("node", node_);  
  getInput("remote_id", remote_id);
  remote_id = "to_be_set";

  tree_pub_ = node_->create_publisher<bf_msgs::msg::MissionCommand>(
    "/" + remote_id + "/mission_tree", 100);

  remote_sub_ = node_->create_subscription<bf_msgs::msg::MissionStatus>(
    "/" + remote_id + "/mission_status", rclcpp::SensorDataQoS(),
    std::bind(&DelegateActionNode::remote_status_callback, this, std::placeholders::_1));  

  getInput("tree", tree);
  // PUBLISH TREE HERE
  
}

void
DelegateActionNode::remote_status_callback(bf_msgs::msg::MissionStatus::UniquePtr msg)
{
  remote_status_ = std::move(msg);

  // if(!remote_itentified_){
  //   remote_id = msg->robot_id;
  //   remote_itentified_ = true;

  //   remote_sub_ = node_->create_subscription<bf_msgs::msg::MissionStatus>(
  //   "/" + remote_id + "/mission_status", rclcpp::SensorDataQoS(),
  //   std::bind(&DelegateActionNode::remote_status_callback, this, std::placeholders::_1));

  // }

  std::cout << "remote status callback" << std::endl;

}

BT::NodeStatus
DelegateActionNode::tick()
{
  // int status = remote_status_->status;

  // switch(status){
  //   case RUNNING:
  //     return BT::NodeStatus::RUNNING;
  //     break;
  //   case SUCCESS:
  //     return BT::NodeStatus::SUCCESS;
  //     break;
  //   case FAILURE:
  //     return BT::NodeStatus::FAILURE;
  //     break;
  //   }

  //   return BT::NodeStatus::FAILURE;

  std::cout << "DelegateActionNode::tick()" << std::endl;
  return BT::NodeStatus::RUNNING;
}

}  // namespace BF

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<BF::DelegateActionNode>("DelegateActionNode");
}
