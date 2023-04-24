#ifndef BF__DELEGATEBT_HPP_
#define BF__DELEGATEBT_HPP_

#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/decorator_node.h"


#include "rclcpp/rclcpp.hpp"

namespace BF
{

class DelegateBT : public BT::DecoratorNode
{
public:

  DelegateBT(const std::string& name, const BT::NodeConfig& conf);

  virtual ~DelegateBT() override = default;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({});
  }

private:

  rclcpp::Node::SharedPtr node_;
  
  virtual BT::NodeStatus tick() override;

  void halt() override;
};

}   // namespace BF

#endif // BF__DELEGATEBT_HPP_