// Copyright (c) 2019 OUXT Polaris
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

/**
 * @file odom_frame_publisher_node.cpp
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief Main function of the odom_frame_publisher node
 * @version 0.1
 * @date 2019-09-23
 *
 * @copyright Copyright (c) 2019
 *
 */

// headers in this package
#include <odom_frame_publisher/odom_frame_publisher_component.hpp>

// headers for ros2
#include <rclcpp/rclcpp.hpp>

// headers in STL
#include <memory>

/**
 * @brief main function
 *
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<odom_frame_publisher::OdomFramePublisherComponent>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
