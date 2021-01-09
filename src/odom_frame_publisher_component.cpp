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
 * @file odom_frame_publisher.cpp
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief Implimentation of the OdomFramePublisher Class
 * @version 0.1
 * @date 2019-09-23
 *
 * @copyright Copyright (c) 2019
 *
 */

// headers in this package
#include <odom_frame_publisher/odom_frame_publisher_component.hpp>

// headers in ros2
#include <rclcpp_components/register_node_macro.hpp>


// headers in stl
#include <memory>
#include <string>

namespace odom_frame_publisher
{
OdomFramePublisherComponent::OdomFramePublisherComponent(const rclcpp::NodeOptions & options)
: Node("odom_frame_publisher", options), buffer_(get_clock()), listener_(buffer_),
  broadcaster_(this)
{
  current_pose_recieved_ = false;
  declare_parameter("current_pose_topic", "current_pose");
  std::string current_pose_topic;
  get_parameter("current_pose_topic", current_pose_topic);
  declare_parameter("current_twist_topic", "current_twist");
  std::string current_twist_topic;
  get_parameter("current_twist_topic", current_twist_topic);
  declare_parameter("odom_frame_id", "odom");
  get_parameter("odom_frame_id", odom_frame_id_);
  declare_parameter("map_frame_id", "map");
  get_parameter("map_frame_id", map_frame_id_);
  declare_parameter("robot_frame_id", "base_link");
  get_parameter("robot_frame_id", robot_frame_id_);
  data_ = boost::circular_buffer<geometry_msgs::msg::TwistStamped>(2);
  current_odom_pose_.header.frame_id = odom_frame_id_;
  current_odom_pose_.header.stamp = get_clock()->now();
  current_odom_pose_.pose.position.x = 0;
  current_odom_pose_.pose.position.y = 0;
  current_odom_pose_.pose.position.z = 0;
  current_odom_pose_.pose.orientation.x = 0;
  current_odom_pose_.pose.orientation.y = 0;
  current_odom_pose_.pose.orientation.z = 0;
  current_odom_pose_.pose.orientation.w = 1;
  odom_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("odom_pose", 1);
  current_twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    current_twist_topic, 1,
    std::bind(&OdomFramePublisherComponent::currentTwistCallback, this, std::placeholders::_1));
  current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    current_pose_topic, 1,
    std::bind(&OdomFramePublisherComponent::currentPoseCallback, this, std::placeholders::_1));
}

void OdomFramePublisherComponent::currentTwistCallback(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  data_.push_back(*msg);
  if (data_.size() == 2 && current_pose_recieved_) {
    current_odom_pose_.header.stamp = current_pose_.header.stamp;
    rclcpp::Time t1 = current_pose_.header.stamp;
    rclcpp::Time t2 = data_[0].header.stamp;
    double duration = t1.seconds() - t2.seconds();
    geometry_msgs::msg::Vector3 orientation;
    orientation.x = (data_[0].twist.angular.x + data_[1].twist.angular.x) * duration * 0.5;
    orientation.y = (data_[0].twist.angular.y + data_[1].twist.angular.y) * duration * 0.5;
    orientation.z = (data_[0].twist.angular.z + data_[1].twist.angular.z) * duration * 0.5;
    geometry_msgs::msg::Quaternion twist_angular_quat =
      quaternion_operation::convertEulerAngleToQuaternion(orientation);
    current_odom_pose_.pose.orientation = quaternion_operation::rotation(
      current_odom_pose_.pose.orientation, twist_angular_quat);
    Eigen::Vector3d trans_vec;
    trans_vec(0) = (data_[0].twist.linear.x + data_[1].twist.linear.x) * duration * 0.5;
    trans_vec(1) = (data_[0].twist.linear.y + data_[1].twist.linear.y) * duration * 0.5;
    trans_vec(2) = (data_[0].twist.linear.z + data_[1].twist.linear.z) * duration * 0.5;
    Eigen::Matrix3d rotation_mat = quaternion_operation::getRotationMatrix(
      current_odom_pose_.pose.orientation);
    trans_vec = rotation_mat * trans_vec;
    current_odom_pose_.pose.position.x = current_odom_pose_.pose.position.x + trans_vec(0);
    current_odom_pose_.pose.position.x = current_odom_pose_.pose.position.x + trans_vec(1);
    current_odom_pose_.pose.position.x = current_odom_pose_.pose.position.x + trans_vec(2);
    odom_pose_pub_->publish(current_odom_pose_);

    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.frame_id = odom_frame_id_;
    transform_stamped.header.stamp = current_pose_.header.stamp;
    transform_stamped.child_frame_id = robot_frame_id_;
    transform_stamped.transform.translation.x = current_odom_pose_.pose.position.x;
    transform_stamped.transform.translation.y = current_odom_pose_.pose.position.y;
    transform_stamped.transform.translation.z = current_odom_pose_.pose.position.z;
    transform_stamped.transform.rotation = current_odom_pose_.pose.orientation;
    broadcaster_.sendTransform(transform_stamped);

    tf2::Transform latest_tf;
    try {
      tf2::Transform odom_pose_tf2;
      tf2::convert(current_odom_pose_.pose, odom_pose_tf2);
      tf2::Quaternion q(current_pose_.pose.orientation.x, current_pose_.pose.orientation.y,
        current_pose_.pose.orientation.z, current_pose_.pose.orientation.w);
      tf2::Transform tmp_tf(q,
        tf2::Vector3(
          current_pose_.pose.position.x, current_pose_.pose.position.y,
          current_pose_.pose.position.z));
      geometry_msgs::msg::PoseStamped tmp_tf_stamped;
      tmp_tf_stamped.header.frame_id = robot_frame_id_;
      tmp_tf_stamped.header.stamp = current_pose_.header.stamp;
      tf2::toMsg(tmp_tf.inverse(), tmp_tf_stamped.pose);
      geometry_msgs::msg::PoseStamped odom_to_map;
      buffer_.transform(tmp_tf_stamped, odom_to_map, odom_frame_id_, tf2::durationFromSec(0.1));
      tf2::convert(odom_to_map.pose, latest_tf);
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "Failed to subtract base to odom transform");
      return;
    }
    geometry_msgs::msg::TransformStamped tmp_tf_stamped;
    tmp_tf_stamped.header.frame_id = map_frame_id_;
    tmp_tf_stamped.header.stamp = current_pose_.header.stamp;
    tmp_tf_stamped.child_frame_id = odom_frame_id_;
    tf2::convert(latest_tf.inverse(), tmp_tf_stamped.transform);
    broadcaster_.sendTransform(tmp_tf_stamped);
  }
}

void OdomFramePublisherComponent::currentPoseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_pose_recieved_ = true;
  current_pose_ = *msg;
}
}  // namespace odom_frame_publisher

RCLCPP_COMPONENTS_REGISTER_NODE(odom_frame_publisher::OdomFramePublisherComponent)
