// Copyright 2023 Intelligent Robotics Lab
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
#include <iostream>

#include "behaviorfleets/deco/DelegateBT.hpp"
#include "rclcpp/rclcpp.hpp"


namespace BF
{

DelegateBT::DelegateBT(const std::string & name, const BT::NodeConfig & conf)
: DecoratorNode(name, conf)
{
}

BT::NodeStatus DelegateBT::tick()
{
  return child_node_->executeTick();
}

void DelegateBT::halt()
{
  BT::DecoratorNode::halt();
}

}   // namespace BF

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<BF::DelegateBT>("DelegateBT");
}
