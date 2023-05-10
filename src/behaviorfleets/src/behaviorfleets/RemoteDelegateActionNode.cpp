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

  // tree_ = create_tree();
  
  timer_ = create_wall_timer(50ms, std::bind(&RemoteDelegateActionNode::control_cycle, this));
}


void
RemoteDelegateActionNode::control_cycle(){


    bf_msgs::msg::MissionStatus msg;

    msg.robot_id = id_;
    msg.status = RUNNING;

    // bool stop = true;

    // BT::NodeStatus status = tree_.rootNode()->executeTick();
    
    // switch(status){
    //   case BT::NodeStatus::RUNNING:
    //     msg.status = RUNNING;
    //     stop = false;
    //     break;
    //   case BT::NodeStatus::SUCCESS:
    //     msg.status = SUCCESS;
    //     working_ = false;
    //     break;
    //   case BT::NodeStatus::FAILURE:
    //     msg.status = FAILURE;
    //     working_ = false;
    //     break;
    //   }  

    // std::cout << "publishing status" << std::endl;
    status_pub_->publish(msg);
    // if(!stop)
    //   rclcpp::spin_some(node_);
    // //else
    //   //  rclcpp::shutdown();   
}

BT::Tree
RemoteDelegateActionNode::create_tree(){

  node_ = rclcpp::Node::make_shared("node");

 
  BT::SharedLibrary loader;
  std::cout << "loader ready" << std::endl;

  BT::BehaviorTreeFactory factory;
  

  std::cout << "LIBRARIES" << std::endl;
  std::cout << "\t-" << loader.getOSName("delegation_node")   << std::endl;


  factory.registerFromPlugin(loader.getOSName("delegation_node"));

  std::cout << "tree nodes registered" << std::endl;
  
  std::string pkgpath = ament_index_cpp::get_package_share_directory("bf_example");
  std::string xml_file = pkgpath + "/bt_xml/example.xml";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node_);
  std::cout << "\t- Blackboard set" << std::endl;
  // BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);
  BT::Tree tree = factory.createTreeFromText(mission_->mission_tree, blackboard);

  return tree;

}

void
RemoteDelegateActionNode::mission_callback(bf_msgs::msg::MissionCommand::UniquePtr msg){
  mission_ = std::move(msg);
  if(!working_) {
    std::cout << "mission received" << std::endl << mission_->mission_tree << std::endl;
    // tree_ = create_tree();
    working_ = true;
  } else {
    std::cout << "mission received but node is busy" << std::endl;
  }
    
  
}

void
RemoteDelegateActionNode::setID(std::string id){
  id_ = id;
}

}  // namespace BF
