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
#include <memory>
#include <fstream>
#include <streambuf>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_cpp/utils/safe_any.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"

bool is_pointer(const std::string & type_name);
std::vector<std::string> check_blackboard(BT::Blackboard::Ptr bb, BT::Blackboard::Ptr bb_cache);

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Read node parameters
  auto node = rclcpp::Node::make_shared("source_tree");
  node->declare_parameter("plugins", std::vector<std::string>({}));
  node->declare_parameter("behavior_tree_xml", "");

  std::vector<std::string> plugins;
  std::string bt_xml_filename;
  node->get_parameter("plugins", plugins);
  node->get_parameter("behavior_tree_xml", bt_xml_filename);

  if (bt_xml_filename == "") {
    RCLCPP_ERROR(node->get_logger(), "BT XML Filename void");
    return -1;
  }

  // Loading plugins from parameters
  BT::SharedLibrary loader;
  BT::BehaviorTreeFactory factory;

  RCLCPP_INFO_STREAM(node->get_logger(), "Loading " << plugins.size() << " plugins" << ":");
  for (const auto & plugin : plugins) {
    RCLCPP_INFO_STREAM(node->get_logger(), "\t" << plugin);
    factory.registerFromPlugin(loader.getOSName(plugin));
  }

  // Read Behavior Tree XML file into an string
  std::string pkgpath = ament_index_cpp::get_package_share_directory("behaviorfleets");
  std::string xml_file = pkgpath + "/bt_xml/" + bt_xml_filename;
  RCLCPP_INFO_STREAM(node->get_logger(), "Loading " << xml_file);
  std::ifstream ifs_xml(xml_file);
  std::string str_xml(
    (std::istreambuf_iterator<char>(ifs_xml)),
    std::istreambuf_iterator<char>());

  // Create Tree from XML readed and init blackboard, inserting XML on it to be used from BT nodes
  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  blackboard->set("bt_xml", str_xml);

  blackboard->set("pkgpath", pkgpath + "/bt_xml/");

  blackboard->set("double", 33.22);
  float * b = new float(20);
  blackboard->set("b", b);
  char * c = new char('c');
  blackboard->set("c", c);
  blackboard->set("entero", 3);

  auto bb_cache = BT::Blackboard::create();
  bb_cache->set("pkgpath", blackboard->get<std::string>("pkgpath"));
  bb_cache->set("double", 33.22);
  bb_cache->set("b", b);
  bb_cache->set("c", c);
  bb_cache->set("entero", 3);

  BT::Tree tree = factory.createTreeFromText(str_xml, blackboard);

  RCLCPP_INFO_STREAM(logger, "\t- Tree created from file");

  rclcpp::Rate rate(10);

  bool finish = false;
  while (!finish && rclcpp::ok()) {
    finish = tree.rootNode()->executeTick() != BT::NodeStatus::RUNNING;
    rclcpp::spin_some(node);
    rate.sleep();

    std::vector<std::string> entries_to_propagate = check_blackboard(
      blackboard, bb_cache, node->get_logger());

    // analyze the blackboard to propagate
    //   std::vector<BT::StringView> string_views = blackboard->getKeys();
    //   std::vector<std::string> keys;
    //   keys.reserve(string_views.size());

    //   for (const auto & string_view : string_views) {
    //     keys.emplace_back(string_view.data(), string_view.size());
    //   }

    //   std::vector<std::string> entries_to_propagate;
    //   for (const std::string & key : keys) {
    //     const auto & value = blackboard->getAny(key);

    //     RCLCPP_INFO_STREAM(logger, "Key: " << key << "; type: " << value->type().name());
    //     if (is_pointer(value->type().name())) {
    //       continue;
    //     }
    //     try {
    //       if ((value->type() == typeid(int)) || (value->type() == typeid(int64_t)) ||
    //         (value->type() == typeid(uint64_t)))
    //       {
    //         int64_t val = value->cast<int64_t>();
    //         RCLCPP_INFO_STREAM(logger, "\t" << val);
    //         int aux;
    //         bb_cache->get(key, aux);
    //         if (aux != val) {
    //           RCLCPP_INFO_STREAM(logger, "cache = " << aux << "actual = " << val);
    //           entries_to_propagate.push_back(key);
    //         }
    //       }
    //       if (value->type() == typeid(double)) {
    //         double val = value->cast<double>();
    //         RCLCPP_INFO_STREAM(logger, "\t" << val);
    //         double aux;
    //         bb_cache->get(key, aux);
    //         if (aux != val) {
    //           RCLCPP_INFO_STREAM(logger, "cache = " << aux << "actual = " << val);
    //           entries_to_propagate.push_back(key);
    //         }
    //       }
    //       if (value->type() == typeid(std::string)) {
    //         std::string val = value->cast<std::string>();
    //         RCLCPP_INFO_STREAM(logger, "\t" << val);
    //         std::string aux;
    //         bb_cache->get(key, aux);
    //         if (aux != val) {
    //           RCLCPP_INFO_STREAM(logger, "propagate");
    //           entries_to_propagate.push_back(key);
    //         }
    //       }
    //     } catch (const boost::bad_any_cast & e) {
    //       std::cerr << "\t- ERROR - Failed to cast " << key << " to its original type: " <<
    //         e.what());
    //     }
    //   }
    //   if (entries_to_propagate.size() > 0) {
    //     RCLCPP_INFO_STREAM(logger, "Propagating updated entries");
    //     for (const std::string & key : entries_to_propagate) {
    //       const auto & value = blackboard->getAny(key);
    //       // TODO: publish entry
    //     }
    //   }

  }

  RCLCPP_INFO_STREAM(node->get_logger(), "Finished ");
  rclcpp::shutdown();
  return 0;
}

bool
is_pointer(const std::string & type_name)
{
  return (type_name.find('*') != std::string::npos) ||
         (type_name.find("ptr") != std::string::npos) ||
         ((type_name.find("P") != std::string::npos) && type_name.size() == 2);
}

std::vector<std::string>
check_blackboard(BT::Blackboard::Ptr bb, BT::Blackboard::Ptr bb_cache, rclcpp::Logger & logger)
{
  std::vector<BT::StringView> string_views = bb->getKeys();
  std::vector<std::string> keys;
  std::vector<std::string> entries_to_propagate;

  keys.reserve(string_views.size());

  for (const auto & string_view : string_views) {
    keys.push_back(string_view.data());
  }

  RCLCPP_INFO_STREAM(logger, keys.size());

  for (const std::string & key : keys) {
    RCLCPP_INFO_STREAM(logger, key);
    const auto & value = bb->getAny(key);
    RCLCPP_INFO_STREAM(logger, "------------------------");
    RCLCPP_INFO_STREAM(logger, "Key: " << key << "; type: " << value->type().name());
    if (is_pointer(value->type().name())) {
      continue;
    }
    try {
      if ((value->type() == typeid(int)) || (value->type() == typeid(int64_t)) ||
        (value->type() == typeid(uint64_t)))
      {
        int64_t val = value->cast<int64_t>();
        RCLCPP_INFO_STREAM(logger, "\t" << val);
        int aux;
        bb_cache->get(key, aux);
        if (aux != val) {
          RCLCPP_INFO_STREAM(logger, "cache = " << aux << "actual = " << val);
          entries_to_propagate.push_back(key);
        }
      }
      if (value->type() == typeid(double)) {
        double val = value->cast<double>();
        RCLCPP_INFO_STREAM(logger, "\t" << val);
        double aux;
        aux = 22;
        bb_cache->get(key, aux);
        if (aux != val) {
          RCLCPP_INFO_STREAM(logger, "cache = " << aux << "actual = " << val);
          entries_to_propagate.push_back(key);
        }
      }
      if (value->type() == typeid(std::string)) {
        std::string val = value->cast<std::string>();
        RCLCPP_INFO_STREAM(logger, "\t" << val);
        std::string aux;
        bb_cache->get(key, aux);
        if (aux != val) {
          RCLCPP_INFO_STREAM(logger, "propagate");
          entries_to_propagate.push_back(key);
        }
      }
    } catch (const boost::bad_any_cast & e) {
      RCLCPP_ERROR_STREAM(
        logger,
        "\t- ERROR - Failed to cast " << key << " to its original type: " << e.what());
    }
  }

  return entries_to_propagate;
}
