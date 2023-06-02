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
#include <boost/any.hpp>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_cpp/utils/safe_any.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"

#include "yaml-cpp/yaml.h"

bool is_pointer(const std::string & type_name);
std::vector<std::string> check_blackboard(BT::Blackboard::Ptr bb, BT::Blackboard::Ptr bb_cache);

int main(int argc, char * argv[])
{
  std::string params_file = "config.yaml";

  if (argc > 1) {
    params_file = std::string(argv[1]);
  }

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("source_tree");

  BT::SharedLibrary loader;
  BT::BehaviorTreeFactory factory;

  factory.registerFromPlugin(loader.getOSName("delegate_action_node"));

  std::string pkgpath = ament_index_cpp::get_package_share_directory("behaviorfleets");

  std::string xml_file;

  try {
    // Load the XML path from the YAML file
    std::cout << "Configuration file: " << params_file << std::endl;
    std::ifstream fin(pkgpath + "/params/" + params_file);
    YAML::Node params = YAML::Load(fin);

    xml_file = pkgpath + params["source_tree"].as<std::string>();

  } catch (YAML::Exception & e) {
    std::cerr << "Error loading YAML file: " << e.what() << std::endl;
    return 1;
  } catch (std::exception & e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
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

  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  std::cout << "\t- Tree created from file" << std::endl;

  rclcpp::Rate rate(10);

  bool finish = false;
  while (!finish && rclcpp::ok()) {
    finish = tree.rootNode()->executeTick() != BT::NodeStatus::RUNNING;
    rclcpp::spin_some(node);
    rate.sleep();

    std::vector<std::string> entries_to_propagate = check_blackboard(blackboard, bb_cache);

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

  //     std::cout << "Key: " << key << "; type: " << value->type().name() << std::endl;
  //     if (is_pointer(value->type().name())) {
  //       continue;
  //     }
  //     try {
  //       if ((value->type() == typeid(int)) || (value->type() == typeid(int64_t)) ||
  //         (value->type() == typeid(uint64_t)))
  //       {
  //         int64_t val = value->cast<int64_t>();
  //         std::cout << "\t" << val << std::endl;
  //         int aux;
  //         bb_cache->get(key, aux);
  //         if (aux != val) {
  //           std::cout << "cache = " << aux << "actual = " << val << std::endl;
  //           entries_to_propagate.push_back(key);
  //         }
  //       }
  //       if (value->type() == typeid(double)) {
  //         double val = value->cast<double>();
  //         std::cout << "\t" << val << std::endl;
  //         double aux;
  //         bb_cache->get(key, aux);
  //         if (aux != val) {
  //           std::cout << "cache = " << aux << "actual = " << val << std::endl;
  //           entries_to_propagate.push_back(key);
  //         }
  //       }
  //       if (value->type() == typeid(std::string)) {
  //         std::string val = value->cast<std::string>();
  //         std::cout << "\t" << val << std::endl;
  //         std::string aux;
  //         bb_cache->get(key, aux);
  //         if (aux != val) {
  //           std::cout << "propagate" << std::endl;
  //           entries_to_propagate.push_back(key);
  //         }
  //       }
  //     } catch (const boost::bad_any_cast & e) {
  //       std::cerr << "\t- ERROR - Failed to cast " << key << " to its original type: " <<
  //         e.what() << std::endl;
  //     }
  //   }
  //   if (entries_to_propagate.size() > 0) {
  //     std::cout << "Propagating updated entries" << std::endl;
  //     for (const std::string & key : entries_to_propagate) {
  //       const auto & value = blackboard->getAny(key);
  //       // TODO: publish entry
  //     }
  //   }

  }

  std::cout << "Finished" << std::endl;
  rclcpp::shutdown();
  return 0;
}

bool is_pointer(const std::string & type_name)
{
  return (type_name.find('*') != std::string::npos) ||
         (type_name.find("ptr") != std::string::npos) ||
         ((type_name.find("P") != std::string::npos) && type_name.size() == 2);
}

std::vector<std::string> check_blackboard(BT::Blackboard::Ptr bb, BT::Blackboard::Ptr bb_cache)
{
  std::vector<BT::StringView> string_views = bb->getKeys();
  std::vector<std::string> keys;
  std::vector<std::string> entries_to_propagate;

  keys.reserve(string_views.size());

  for (const auto& string_view : string_views) {
    keys.push_back(string_view.data());
  }

  std::cout << keys.size() << std::endl;

  for (const std::string & key : keys) {
    std::cout << key << std::endl;
    const auto& value = bb->getAny(key);
    std::cout << "------------------------" << std::endl;
    std::cout << "Key: " << key << "; type: " << value->type().name() << std::endl;
    if (is_pointer(value->type().name())) {
        continue;
    }
    try {
      if ((value->type() == typeid(int)) || (value->type() == typeid(int64_t)) ||
        (value->type() == typeid(uint64_t))) {
          int64_t val = value->cast<int64_t>();
          std::cout << "\t" << val << std::endl;
          int aux;
          bb_cache->get(key, aux);
          if (aux != val) {
            std::cout << "cache = " << aux << "actual = " << val <<std::endl;
            entries_to_propagate.push_back(key);
          }
      }
      if (value->type() == typeid(double)) {
        double val = value->cast<double>();
        std::cout << "\t" << val << std::endl;
        double aux;
        aux = 22;
        bb_cache->get(key, aux);
        if (aux != val) {
          std::cout << "cache = " << aux << "actual = " << val <<std::endl;
          entries_to_propagate.push_back(key);
        }
      }
      if (value->type() == typeid(std::string)) {
        std::string val = value->cast<std::string>();
        std::cout << "\t" << val << std::endl;
        std::string aux;
        bb_cache->get(key, aux);
        if (aux != val) {
          std::cout << "propagate" << std::endl;
          entries_to_propagate.push_back(key);
        }
      }
    } catch (const boost::bad_any_cast& e) {
      std::cerr << "\t- ERROR - Failed to cast " << key << " to its original type: " << e.what() << std::endl;
    }
  }
  return entries_to_propagate;
}
