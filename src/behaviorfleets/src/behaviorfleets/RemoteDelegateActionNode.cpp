#include <string>
#include <iostream>



#include "behaviortree_cpp/behavior_tree.h"
#include "bf_msgs/msg/mission_status.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "behaviortree_cpp/utils/shared_library.h"
#include "rclcpp/rclcpp.hpp"

#include "behaviorfleets/RemoteDelegateActionNode.hpp"


namespace BF
{

RemoteDelegateActionNode::RemoteDelegateActionNode()
: Node("RemoteDelegateAN"),
id_("remote")
{
  init();
}

RemoteDelegateActionNode::RemoteDelegateActionNode(const std::string id)
: Node("RemoteDelegateAN"),
id_(id)
{
  init();
}


void
RemoteDelegateActionNode::init(){

  using namespace std::chrono_literals;
  
  mission_sub_ = create_subscription<bf_msgs::msg::MissionCommand>(
    "/" + id_ + "/mission_command", rclcpp::SensorDataQoS(),
    std::bind(&RemoteDelegateActionNode::mission_callback, this, std::placeholders::_1));

  std::cout << "subscribed to " << "/" + id_ + "/mission_command"<< std::endl;

  status_pub_ = create_publisher<bf_msgs::msg::MissionStatus>(
    "/" + id_ + "/mission_status", 10);

  timer_ = create_wall_timer(50ms, std::bind(&RemoteDelegateActionNode::control_cycle, this));

  this->declare_parameter("plugins",std::vector<std::string>());
 
}

void
RemoteDelegateActionNode::mission_callback(bf_msgs::msg::MissionCommand::UniquePtr msg){
  mission_ = std::move(msg);
  if(!working_) {
    std::cout << "mission received" << std::endl << mission_->mission_tree << std::endl;
    tree_ = create_tree();
    working_ = true;
  } else {
    std::cout << "mission received but node is busy" << std::endl;
  }
  
}

void
RemoteDelegateActionNode::control_cycle(){


    bf_msgs::msg::MissionStatus msg;

    msg.robot_id = id_;
    msg.status = RUNNING;

    if(working_) {
      BT::NodeStatus status = tree_.rootNode()->executeTick();
      switch(status) {
        case BT::NodeStatus::RUNNING:
          msg.status = RUNNING;
          std::cout << "RUNNING" << std::endl;
          break;
        case BT::NodeStatus::SUCCESS:
          msg.status = SUCCESS;
          std::cout << "SUCCESS" << std::endl;
          working_ = false;
          break;
        case BT::NodeStatus::FAILURE:
          msg.status = FAILURE;
          std::cout << "FAILURE" << std::endl;
          working_ = false;
          break;
      }
    } else {
      msg.status = IDLE;
    }
   
    status_pub_->publish(msg);
  
    
    // if(working_)
      // rclcpp::spin_some(node_);
    //else
      //  rclcpp::shutdown();  
     
}

BT::Tree
RemoteDelegateActionNode::create_tree(){

  BT::SharedLibrary loader;
  BT::BehaviorTreeFactory factory;
  
  auto plugins = this->get_parameter("plugins").as_string_array();

  for(auto plugin : plugins)
    factory.registerFromPlugin(loader.getOSName(plugin));
    
      
  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", shared_from_this());
  BT::Tree tree = factory.createTreeFromText(mission_->mission_tree, blackboard);

  std::cout << "\t- Tree created" << std::endl;

  return tree;

}

void
RemoteDelegateActionNode::setID(std::string id){
  id_ = id;
}


}  // namespace BF
