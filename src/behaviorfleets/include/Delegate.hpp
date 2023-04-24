#ifndef BF__DELEGATE_HPP_
#define BF__DELEGATE_HPP_

#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/decorator_node.h"


#include "rclcpp/rclcpp.hpp"

namespace BF
{

class Delegate : public BT::ActionNodeBase
{
public:

  Delegate(
    const std::string& name,
    const BT::NodeConfig& conf);


  void mission_callback(bf_msgs::msg::MissionCommand::UniquePtr msg);
  
  BT::Tree create_tree();
  
  void control_cycle();

  void halt() override
  {}

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({});
  }

private:

  rclcpp::Node::SharedPtr node_; // pointer to the node running the delegated BT
  
  virtual BT::NodeStatus tick() override;

  static const int FAILURE = 0;
  static const int SUCCESS = 1;
  static const int RUNNING = 2;

  bf_msgs::msg::MissionCommand::UniquePtr mission_;
  std::string robot_id_; // identifier of the delegate robot

  rclcpp::Publisher<bf_msgs::msg::MissionStatus>::SharedPtr status_pub_; 
  rclcpp::Subscription<bf_msgs::msg::MissionCommand>::SharedPtr mission_sub_;
  
  BT::Tree tree_; // delegated BT
  rclcpp::TimerBase::SharedPtr timer_;

};

}   // namespace BF

#endif // BF__DELEGATE_HPP_