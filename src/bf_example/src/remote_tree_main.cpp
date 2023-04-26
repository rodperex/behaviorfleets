#include <string>
#include <memory>


#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"

#include "behaviorfleets/RemoteDelegateActionNode.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // BF::RemoteDelegateActionNode node();
  // node.setID("example");

  // auto node = std::make_shared<BF::RemoteDelegateActionNode>(); //("example");
  

  rclcpp::shutdown();
  return 0;

}