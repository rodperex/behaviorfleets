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

#ifndef BEHAVIORFLEETS__DECO__DELEGATEBT_HPP_
#define BEHAVIORFLEETS__DECO__DELEGATEBT_HPP_

#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/decorator_node.h"


#include "rclcpp/rclcpp.hpp"

namespace BF
{

class DelegateBT : public BT::DecoratorNode
{
public:
  DelegateBT(const std::string& name, const BT::NodeConfig& conf);

  ~DelegateBT() override = default;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({});
  }

private:
  rclcpp::Node::SharedPtr node_;

  BT::NodeStatus tick() override;

  void halt() override;
};

}   // namespace BF

#endif  // BEHAVIORFLEETS__DECO__DELEGATEBT_HPP_
