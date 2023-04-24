#ifndef BF__DELEGATION_HPP_
#define BF__DELEGATION_HPP_

#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/decorator_node.h"
#include "bf_msgs/msg/mission_command.hpp"
#include "bf_msgs/msg/mission_status.hpp"

#include "rclcpp/rclcpp.hpp"

namespace BF
{

class DelegationNode : public BT::DecoratorNode
{
public:

  DelegationNode(const std::string& name, const BT::NodeConfig& conf);

  virtual ~DelegationNode() override = default;

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<int>(MISSION, "Mission to delegate")};
  }

  void child_status_callback(bf_msgs::msg::MissionStatus::UniquePtr msg);

private:

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<bf_msgs::msg::MissionCommand>::SharedPtr tree_pub_;
  rclcpp::Subscription<bf_msgs::msg::MissionStatus>::SharedPtr child_sub_;
  
  bf_msgs::msg::MissionStatus::UniquePtr child_status_;
  std::string child_id;
  bool child_itentified_ = false;

  bool read_tree_from_port_;
  static constexpr const char* MISSION = "";

  virtual BT::NodeStatus tick() override;

  void halt() override;
};

}   // namespace BF

#endif // BF__DELEGATION_HPP_