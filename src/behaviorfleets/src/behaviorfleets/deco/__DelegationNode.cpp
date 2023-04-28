
#include <string>
#include <iostream>

#include "behaviorfleets/DelegationNode.hpp"
#include "bf_msgs/msg/mission_command.hpp"
#include "bf_msgs/msg/mission_status.hpp"
#include "rclcpp/rclcpp.hpp"


namespace BF
{

DelegationNode::DelegationNode(const std::string &name, const BT::NodeConfig &conf) : 
DecoratorNode(name, conf),
read_tree_from_port_(true)
{
  std::string tree;

  config().blackboard->get("node", node_);  

  tree_pub_ = node_->create_publisher<bf_msgs::msg::MissionCommand>("/mission_tree", 100);

  getInput("tree", tree);
  // PUBLISH TREE HERE
  
}

void
DelegationNode::child_status_callback(bf_msgs::msg::MissionStatus::UniquePtr msg)
{
  child_status_ = std::move(msg);

  if(!child_itentified_){
    child_id = msg->robot_id;
    child_itentified_ = true;

    child_sub_ = node_->create_subscription<bf_msgs::msg::MissionStatus>(
    "/" + child_id + "/mission_status", rclcpp::SensorDataQoS(),
    std::bind(&DelegationNode::child_status_callback, this, std::placeholders::_1));

    
  }

}

BT::NodeStatus DelegationNode::tick()
{
    return BT::NodeStatus::SUCCESS;
}

void DelegationNode::halt()
{
  DecoratorNode::halt();
}

}   // namespace BF

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<BF::DelegationNode>("Delegation");
}
