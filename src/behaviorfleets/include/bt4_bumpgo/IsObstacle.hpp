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

#ifndef BT4_BUMPGO__ISOBSTACLE_HPP_
#define BT4_BUMPGO__ISOBSTACLE_HPP_

#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bt4_bumpgo
{

class IsObstacle : public BT::ConditionNode
{
public:
  explicit IsObstacle(
    const std::string & xml_tag_name,
    const BT::NodeConfig & conf);

  BT::NodeStatus tick();


  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<double>("distance"),
        BT::OutputPort<int>("direction")
      });
  }

  void laser_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time last_reading_time_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  sensor_msgs::msg::LaserScan::UniquePtr last_scan_;
  static const int LEFT = 1;
  static const int RIGHT = 2;
};

}  // namespace bt4_bumpgo

#endif  // BT4_BUMPGO__ISOBSTACLE_HPP_
