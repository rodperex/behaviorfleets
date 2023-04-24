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
  mission_sub_ = create_subscription<bf_msgs::msg::MissionCommand>(
    "/mission_command", rclcpp::SensorDataQoS(),
    std::bind(&Test::mission_callback, this, std::placeholders::_1));

  status_pub_ = create_publisher<bf_msgs::msg::MissionStatus>("/" + id_ +"/mission_status", 10);

  auto node = rclcpp::Node::make_shared("example");
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
  blackboard->set("node", node);
  std::cout << "\t- Blackboard set" << std::endl;
  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  
  rclcpp::Rate rate(10);
 
  bool finish = false;
  while (!finish && rclcpp::ok()) {
    finish = tree.rootNode()->executeTick() != BT::NodeStatus::RUNNING;
    
    rclcpp::spin_some(node);
    rate.sleep();
  }

}

void
Test::mission_callback(bf_msgs::msg::MissionCommand::UniquePtr msg){
  mission_ = std::move(msg);
}

}  // namespace bf_example
