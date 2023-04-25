#include <string>
#include <memory>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"

#include "RemoteDelegateActionNode.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  BF::RemoteDelegateActionNode node();

  // auto node = std::make_shared<BF::RemoteDelegateActionNode>(); 

  // auto node = rclcpp::Node::make_shared("example");
  // std::cout << "running main..." << std::endl;

  
  // BT::SharedLibrary loader;
  // std::cout << "loader ready" << std::endl;

  // BT::BehaviorTreeFactory factory;
  

  // std::cout << "LIBRARIES" << std::endl;
  // std::cout << "\t-" << loader.getOSName("delegation_node")   << std::endl;


  // factory.registerFromPlugin(loader.getOSName("delegation_node"));

  // std::cout << "tree nodes registered" << std::endl;
  
  // std::string pkgpath = ament_index_cpp::get_package_share_directory("bf_example");
  // std::string xml_file = pkgpath + "/bt_xml/example.xml";

  // auto blackboard = BT::Blackboard::create();
  // blackboard->set("node", node);
  // std::cout << "\t- Blackboard set" << std::endl;
  // BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  
  // rclcpp::Rate rate(10);
 
  // bool finish = false;
  // while (!finish && rclcpp::ok()) {
  //   finish = tree.rootNode()->executeTick() != BT::NodeStatus::RUNNING;
    
  //   rclcpp::spin_some(node);
  //   rate.sleep();
  // }

  rclcpp::shutdown();
  // return 0;
  

  // BT::Tree tree = factory.createTreeFromFile(xml_file);
  // tree.tickWhileRunning();
}