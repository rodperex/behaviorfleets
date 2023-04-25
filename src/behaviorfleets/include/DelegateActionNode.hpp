#ifndef BF__DELEGATEACTIONNODE_HPP_
#define BF__DELEGATEACTIONNNODE_HPP_

#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/decorator_node.h"

#include "bf_msgs/msg/mission_command.hpp"
#include "bf_msgs/msg/mission_status.hpp"


#include "rclcpp/rclcpp.hpp"

namespace BF
{

class DelegateActionNode : public BT::ActionNodeBase
{
public:

  DelegateActionNode(
    const std::string& name,
    const BT::NodeConfig& conf);


  void child_status_callback(bf_msgs::msg::MissionStatus::UniquePtr msg);

  
  void halt() override
  {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<int>(MISSION, "Mission to delegate")};;
  }

private:

  static constexpr const char* MISSION = "";

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<bf_msgs::msg::MissionCommand>::SharedPtr tree_pub_;
  rclcpp::Subscription<bf_msgs::msg::MissionStatus>::SharedPtr child_sub_;
  
  bf_msgs::msg::MissionStatus::UniquePtr child_status_;
  std::string child_id;
  bool child_itentified_ = false;

  static const int FAILURE = 0;
  static const int SUCCESS = 1;
  static const int RUNNING = 2;

  bool read_tree_from_port_;

  virtual BT::NodeStatus tick() override;
  

};

}   // namespace BF

#endif // BF__DELEGATE_HPP_