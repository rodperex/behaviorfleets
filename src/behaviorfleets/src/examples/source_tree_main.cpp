#include <string>
#include <memory>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"

#include "yaml-cpp/yaml.h"
#include <fstream>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("example");

  BT::SharedLibrary loader;
  BT::BehaviorTreeFactory factory;
  
  // factory.registerFromPlugin(loader.getOSName("delegate_bt_node"));
  factory.registerFromPlugin(loader.getOSName("delegate_action_node"));

  std::string pkgpath = ament_index_cpp::get_package_share_directory("behaviorfleets");

  std::string xml_file, xml_file_remote_1, remote_tree_1, remote_id_1;
  
  try {
    // Load the XML path from the YAML file
    std::ifstream fin(pkgpath + "/params/config.yaml");
    YAML::Node params = YAML::Load(fin);

    xml_file = pkgpath + params["tree"].as<std::string>();
    xml_file_remote_1 = pkgpath + params["remote_tree_1"].as<std::string>();
    // remote_id_1 = params["remote_id_1"].as<std::string>();
    
    // std::ifstream file(xml_file_remote_1);
    // std::ostringstream contents_stream;
    
    // contents_stream << file.rdbuf();
    // remote_tree_1 = contents_stream.str();
  } catch (YAML::Exception& e) {
    std::cerr << "Error loading YAML file: " << e.what() << std::endl;
    return 1;
  }

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);  
  blackboard->set("pkgpath", pkgpath + "/bt_xml/");
  
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