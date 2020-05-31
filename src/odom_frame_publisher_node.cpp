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

// headers for ros
#include <rclcpp/rclcpp.hpp>

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
