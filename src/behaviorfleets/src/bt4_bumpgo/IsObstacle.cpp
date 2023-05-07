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
#include <utility>

#include "bt4_bumpgo/IsObstacle.hpp"

#include "behaviortree_cpp/behavior_tree.h"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bt4_bumpgo
{

using namespace std::chrono_literals;
using namespace std::placeholders;

IsObstacle::IsObstacle(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  laser_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    "/input_scan", 100, std::bind(&IsObstacle::laser_callback, this, _1));

  last_reading_time_ = node_->now();
}

void
IsObstacle::laser_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
{
  last_scan_ = std::move(msg);
}

BT::NodeStatus
IsObstacle::tick()
{
  if (last_scan_ == nullptr) {
    return BT::NodeStatus::FAILURE;
  }

  size_t pos;

  double distance = 1.0;
  getInput("distance", distance);
  

  for(pos = 0; pos < last_scan_->ranges.size()/2; pos++){
    if(last_scan_->ranges[pos] < distance){
      setOutput("direction", 2);
      return BT::NodeStatus::SUCCESS;
      break;
    }
  }

  for(pos = (last_scan_->ranges.size()/2 + 1); pos < last_scan_->ranges.size(); pos++){
    if(last_scan_->ranges[pos] < distance){
      setOutput("direction", 1);
      return BT::NodeStatus::SUCCESS;
    }
  }

  return BT::NodeStatus::FAILURE;
  
}

}  // namespace bt4_bumpgo

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt4_bumpgo::IsObstacle>("IsObstacle");
}
