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

#ifndef ODOM_FRAME_PUBLISHER__ODOM_FRAME_PUBLISHER_COMPONENT_HPP_
#define ODOM_FRAME_PUBLISHER__ODOM_FRAME_PUBLISHER_COMPONENT_HPP_

/**
 * @file odom_frame_publisher.h
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief Definition of the OdomFramePublisher Class
 * @version 0.1
 * @date 2019-09-23
 *
 * @copyright Copyright (c) 2019
 *
 */

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define ODOM_FRAME_PUBLISHER_ODOM_FRAME_PUBLISHER_COMPONENT_EXPORT __attribute__((dllexport))
#define ODOM_FRAME_PUBLISHER_ODOM_FRAME_PUBLISHER_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define ODOM_FRAME_PUBLISHER_ODOM_FRAME_PUBLISHER_COMPONENT_EXPORT __declspec(dllexport)
#define ODOM_FRAME_PUBLISHER_ODOM_FRAME_PUBLISHER_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef ODOM_FRAME_PUBLISHER_ODOM_FRAME_PUBLISHER_COMPONENT_BUILDING_DLL
#define ODOM_FRAME_PUBLISHER_ODOM_FRAME_PUBLISHER_COMPONENT_PUBLIC \
  ODOM_FRAME_PUBLISHER_ODOM_FRAME_PUBLISHER_COMPONENT_EXPORT
#else
#define ODOM_FRAME_PUBLISHER_ODOM_FRAME_PUBLISHER_COMPONENT_PUBLIC \
  ODOM_FRAME_PUBLISHER_ODOM_FRAME_PUBLISHER_COMPONENT_IMPORT
#endif
#define ODOM_FRAME_PUBLISHER_ODOM_FRAME_PUBLISHER_COMPONENT_PUBLIC_TYPE \
  ODOM_FRAME_PUBLISHER_ODOM_FRAME_PUBLISHER_COMPONENT_PUBLIC
#define ODOM_FRAME_PUBLISHER_ODOM_FRAME_PUBLISHER_COMPONENT_LOCAL
#else
#define ODOM_FRAME_PUBLISHER_ODOM_FRAME_PUBLISHER_COMPONENT_EXPORT \
  __attribute__((visibility("default")))
#define ODOM_FRAME_PUBLISHER_ODOM_FRAME_PUBLISHER_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define ODOM_FRAME_PUBLISHER_ODOM_FRAME_PUBLISHER_COMPONENT_PUBLIC \
  __attribute__((visibility("default")))
#define ODOM_FRAME_PUBLISHER_ODOM_FRAME_PUBLISHER_COMPONENT_LOCAL \
  __attribute__((visibility("hidden")))
#else
#define ODOM_FRAME_PUBLISHER_ODOM_FRAME_PUBLISHER_COMPONENT_PUBLIC
#define ODOM_FRAME_PUBLISHER_ODOM_FRAME_PUBLISHER_COMPONENT_LOCAL
#endif
#define ODOM_FRAME_PUBLISHER_ODOM_FRAME_PUBLISHER_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

// Headers in ROS
#include <quaternion_operation/quaternion_operation.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/impl/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

// Headers in Boost
#include <boost/circular_buffer.hpp>

// Headers in STL
#include <string>

namespace odom_frame_publisher
{
class OdomFramePublisherComponent : public rclcpp::Node
{
public:
  ODOM_FRAME_PUBLISHER_ODOM_FRAME_PUBLISHER_COMPONENT_PUBLIC
  explicit OdomFramePublisherComponent(const rclcpp::NodeOptions & options);

private:
  void currentTwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void currentPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  geometry_msgs::msg::PoseStamped current_odom_pose_;
  geometry_msgs::msg::PoseStamped current_pose_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  tf2_ros::TransformBroadcaster broadcaster_;
  bool current_pose_recieved_;
  std::string odom_frame_id_;
  std::string map_frame_id_;
  std::string robot_frame_id_;
  boost::circular_buffer<geometry_msgs::msg::TwistStamped> data_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr odom_pose_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr current_twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
};
}  // namespace odom_frame_publisher

#endif  //  ODOM_FRAME_PUBLISHER__ODOM_FRAME_PUBLISHER_COMPONENT_HPP_
