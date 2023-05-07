// Copyright 2021 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <memory>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
// #include "behaviortree_cpp/loggers/bt_zmq_publisher.h"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"

// #include "bt4_bumpgo/Back.hpp"
// #include "bt4_bumpgo/Forward.hpp"
// #include "bt4_bumpgo/Turn.hpp"
// #include "bt4_bumpgo/IsObstacle.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("bumpgo_node");
  std::cout << "bt4_bumpgo_main" << std::endl;

  
  BT::SharedLibrary loader;
  std::cout << "loader ready" << std::endl;

  BT::BehaviorTreeFactory factory;
  
  // factory.registerNodeType<bt4_bumpgo::Back>("Back");
  // factory.registerNodeType<bt4_bumpgo::Forward>("Forward");
  // factory.registerNodeType<bt4_bumpgo::IsObstacle>("IsObstacle");
  // factory.registerNodeType<bt4_bumpgo::Turn>("Turn");

  std::cout << "LIBRARIES" << std::endl;
  std::cout << "\t-" << loader.getOSName("forward_node")     << std::endl;
  std::cout << "\t-" << loader.getOSName("back_node")        << std::endl;
  std::cout << "\t-" << loader.getOSName("turn_node")        << std::endl;
  std::cout << "\t-" << loader.getOSName("is_obstacle_node") << std::endl;

  factory.registerFromPlugin(loader.getOSName("forward_node"));
  factory.registerFromPlugin(loader.getOSName("back_node"));
  factory.registerFromPlugin(loader.getOSName("turn_node"));
  factory.registerFromPlugin(loader.getOSName("is_obstacle_node"));

  std::cout << "tree nodes registered" << std::endl;
  
  std::string pkgpath = ament_index_cpp::get_package_share_directory("bt4_bumpgo");
  std::string xml_file = pkgpath + "/behavior_tree_xml/bumpgo.xml";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  std::cout << "\t- Blackboard set" << std::endl;
  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);
  // std::cout << "\t- BT created" << std::endl;

  //auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, 1666, 1667);

  
  rclcpp::Rate rate(10);
  /*

  Hace listener para añadirle los nodos a spinear. 
  Añadir métodos para matar 1 o todos
  Basarme en transform_listener
  bt_listener(node->shared_from_this(),xml,pth)

  */
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

/*

This is a C++ program that creates and runs a behavior tree using the BehaviorTree.CPP library. The behavior tree is defined in an XML file, which is loaded and executed by the program. The program also initializes a ROS 2 node using the rclcpp library, and publishes the execution status of the behavior tree using a ZMQ (ZeroMQ) publisher.

Here is a brief explanation of the main parts of the code:

The main function initializes the ROS 2 node and sets up the behavior tree factory and loader.

The behavior tree factory is used to register the behavior tree nodes that are defined in shared libraries. In this case, the program registers four nodes called forward_bt_node, back_bt_node, turn_bt_node, and is_obstacle_bt_node. These nodes are defined in shared libraries that are loaded dynamically at runtime using the SharedLibrary class.

The xml_file variable contains the path to the XML file that defines the behavior tree. This file is located in the behavior_tree_xml directory inside the bt_bumpgo package, which is obtained using the ament_index_cpp::get_package_share_directory function.

The blackboard variable is a shared pointer to a Blackboard object, which is used to store data that can be accessed by the behavior tree nodes. In this case, the node object, which represents the ROS 2 node, is stored in the blackboard.

The behavior tree is created using the factory.createTreeFromFile function, which reads the XML file and returns a Tree object that represents the behavior tree.

A ZMQ publisher is created using the BT::PublisherZMQ class, which publishes the execution status of the behavior tree on a specified port. The publisher is initialized with the tree object, a message buffer size of 10, and two port numbers (1666 and 1667) for publishing and subscribing, respectively.

The while loop executes the behavior tree in a loop until the root node returns a status of SUCCESS, FAILURE, or IDLE. The executeTick function is called on the root node of the behavior tree to execute one tick of the tree. The spin_some function is called on the ROS 2 node to process any pending messages, and the rate.sleep function is called to wait for a specified amount of time (in this case, 100ms) before repeating the loop.

Finally, the rclcpp::shutdown function is called to shutdown the ROS 2 node and free any resources that were allocated by the program.


The ZMQ (ZeroMQ) publisher is being used in this code to publish the execution status of the behavior tree. This can be useful for monitoring the behavior of the system and diagnosing any problems that may occur during execution.

The BT::PublisherZMQ class provided by the BehaviorTree.CPP library allows the behavior tree to publish its status on a specified port using the ZeroMQ messaging protocol. This can be useful in situations where the behavior tree is running on a separate process or machine from the monitoring system, or where multiple monitoring systems need to receive the status information.

By publishing the execution status of the behavior tree on a ZMQ socket, other programs can subscribe to the socket and receive real-time updates on the state of the behavior tree. This can help with debugging and monitoring the behavior of the system, as well as providing feedback on how to improve the behavior tree.

*/