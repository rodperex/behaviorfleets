#include <string>
#include <iostream>


#include "Test.hpp"

#include "behaviortree_cpp/behavior_tree.h"
#include "bf_msgs/msg/mission_status.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "behaviortree_cpp/utils/shared_library.h"
#include "rclcpp/rclcpp.hpp"

namespace bf_example
{

Test::Test()
: Node("test"),
id_("test")
{
  using namespace std::chrono_literals;
  
  mission_sub_ = create_subscription<bf_msgs::msg::MissionCommand>(
    "/mission_command", rclcpp::SensorDataQoS(),
    std::bind(&Test::mission_callback, this, std::placeholders::_1));

  status_pub_ = create_publisher<bf_msgs::msg::MissionStatus>("/" + id_ +"/mission_status", 10);

  tree_ = create_tree();
 
  timer_ = create_wall_timer(50ms, std::bind(&Test::control_cycle, this));
}

void
Test::control_cycle(){

    bf_msgs::msg::MissionStatus msg;
    msg.robot_id = id_;
    bool stop = true;

    BT::NodeStatus status = tree_.rootNode()->executeTick();
    
    switch(status){
      case BT::NodeStatus::RUNNING:
        msg.status = RUNNING;
        stop = false;
        break;
      case BT::NodeStatus::SUCCESS:
        msg.status = SUCCESS;
        break;
      case BT::NodeStatus::FAILURE:
        msg.status = FAILURE;
        break;
      }  

    status_pub_->publish(msg);
    if(!stop)
      rclcpp::spin_some(node_);
    else
      rclcpp::shutdown();   
}

BT::Tree
Test::create_tree(){

  node_ = rclcpp::Node::make_shared("example");
  std::cout << "running main..." << std::endl;

  
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
  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  return tree;

}

void
Test::mission_callback(bf_msgs::msg::MissionCommand::UniquePtr msg){
  mission_ = std::move(msg);
}

}  // namespace bf_example
