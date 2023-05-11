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


  void remote_status_callback(bf_msgs::msg::MissionStatus::UniquePtr msg);

  
  void halt() override
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("remote_tree"),
      BT::InputPort<char*>("remote_id"),
    };
  }

private:

  static constexpr const char* MISSION = "";

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<bf_msgs::msg::MissionCommand>::SharedPtr tree_pub_;
  rclcpp::Subscription<bf_msgs::msg::MissionStatus>::SharedPtr remote_sub_;
  
  bf_msgs::msg::MissionStatus::UniquePtr remote_status_;
  std::string remote_id_, remote_tree_;
  bool remote_itentified_ = false;

  static const int FAILURE = 0;
  static const int SUCCESS = 1;
  static const int RUNNING = 2;  

  virtual BT::NodeStatus tick() override;
  

};

}   // namespace BF

#endif // BF__DELEGATE_HPP_