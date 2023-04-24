#include <string>
#include <iostream>


#include "DelegateActionNode.hpp"

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

  config().blackboard->get("node", node_);  

  tree_pub_ = node_->create_publisher<bf_msgs::msg::MissionCommand>("/mission_tree", 100);

  getInput("tree", tree);
  // PUBLISH TREE HERE
  
}

void
DelegateActionNode::child_status_callback(bf_msgs::msg::MissionStatus::UniquePtr msg)
{
  child_status_ = std::move(msg);

  if(!child_itentified_){
    child_id = msg->robot_id;
    child_itentified_ = true;

    child_sub_ = node_->create_subscription<bf_msgs::msg::MissionStatus>(
    "/" + child_id + "/mission_status", rclcpp::SensorDataQoS(),
    std::bind(&DelegateActionNode::child_status_callback, this, std::placeholders::_1));

  }

}

BT::NodeStatus
DelegateActionNode::tick()
{
  int status = child_status_->status;

  switch(status){
    case RUNNING:
      return BT::NodeStatus::RUNNING;
      break;
    case SUCCESS:
      return BT::NodeStatus::SUCCESS;
      break;
    case FAILURE:
      return BT::NodeStatus::FAILURE;
      break;
    }

    return BT::NodeStatus::FAILURE;
}

}  // namespace BF

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<BF::DelegateActionNode>("DelegateActionNode");
}
