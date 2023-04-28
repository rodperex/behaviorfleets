#include <string>
#include <memory>



#include "rclcpp/rclcpp.hpp"

#include "behaviorfleets/RemoteDelegateActionNode.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // auto node = std::make_shared<BF::RemoteDelegateActionNode>("testnode"); 
  
  rclcpp::shutdown();
  return 0;

}