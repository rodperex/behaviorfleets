
#include <string>
#include <iostream>

#include "DelegateBT.hpp"
#include "rclcpp/rclcpp.hpp"


namespace BF
{

DelegateBT::DelegateBT(const std::string &name, const BT::NodeConfig &conf) : 
DecoratorNode(name, conf)
{

}

BT::NodeStatus DelegateBT::tick()
{
    return child_node_->executeTick();
}

void DelegateBT::halt()
{
  BT::DecoratorNode::halt();
}

}   // namespace BF

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<BF::DelegateBT>("DelegateBT");
}
