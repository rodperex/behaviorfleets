#include <string>
#include <memory>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);


  auto node = rclcpp::Node::make_shared("example");
  std::cout << "running main..." << std::endl;

  
  BT::SharedLibrary loader;
  std::cout << "loader ready" << std::endl;

  BT::BehaviorTreeFactory factory;
  

  std::cout << "LIBRARIES" << std::endl;
  std::cout << "\t-" << loader.getOSName("delegate_bt_node")   << std::endl;
  std::cout << "\t-" << loader.getOSName("delegate_action_node")   << std::endl;


  factory.registerFromPlugin(loader.getOSName("delegate_bt_node"));
  factory.registerFromPlugin(loader.getOSName("delegate_action_node"));

  std::cout << "tree nodes registered" << std::endl;
  
  std::string pkgpath = ament_index_cpp::get_package_share_directory("behaviorfleets");
  std::string xml_file = pkgpath + "/bt_xml/a.xml";

  std::cout << "\t- XML: " << xml_file << std::endl;

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  std::cout << "\t- Blackboard set" << std::endl;
  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  std::cout << "\t- Tree created from file" << std::endl;

  
  rclcpp::Rate rate(10);
 
  bool finish = false;
  while (!finish && rclcpp::ok()) {
    finish = tree.rootNode()->executeTick() != BT::NodeStatus::RUNNING;
    
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
  

  // BT::Tree tree = factory.createTreeFromFile(xml_file);
  // tree.tickWhileRunning();
}