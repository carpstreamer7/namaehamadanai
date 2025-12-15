// Copyright 2025 Junko Morofuji
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

// Reference:
// https://github.com/ros-planning/moveit2_tutorials/blob
// /a547cf49ff7d1fe16a93dfe020c6027bcb035b51/doc/move_group_interface
// /src/move_group_interface_tutorial.cpp

//自分で追加
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
//ここまで


#include <cmath>

#include "angles/angles.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_and_place");

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
 
 
  auto move_group_arm_node = rclcpp::Node::make_shared("move_group_arm_node", node_options);
 
  auto move_group_gripper_node = rclcpp::Node::make_shared("move_group_gripper_node", node_options);
 
  // For current state monitor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_arm_node);
  executor.add_node(move_group_gripper_node);
  std::thread([&executor]() {executor.spin();}).detach();

  //追加した。
  std::string target_frame;
  double place_x, place_y, place_z;
  //ここまで
  
  move_group_arm.setMaxVelocityScalingFactor(1.0);
  move_group_arm.setMaxAccelerationScalingFactor(1.0);
  move_group_gripper.setMaxVelocityScalingFactor(1.0);
  move_group_gripper.setMaxAccelerationScalingFactor(1.0);


  //TF
  tf2_ros::Buffer tf_buffer(move_group_arm_node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

   // MoveIt
  MoveGroupInterface move_group_arm(move_group_arm_node, "arm");
  MoveGroupInterface move_group_gripper(move_group_gripper_node, "gripper");

  move_group_arm.setMaxVelocityScalingFactor(1.0);
  move_group_arm.setMaxAccelerationScalingFactor(1.0);
  move_group_gripper.setMaxVelocityScalingFactor(1.0);
  move_group_gripper.setMaxAccelerationScalingFactor(1.0);

  auto gripper_joint_values = move_group_gripper.getCurrentJointValues();
  const double GRIPPER_DEFAULT = 0.0;
  const double GRIPPER_OPEN = angles::from_degrees(60.0);
  const double GRIPPER_CLOSE = angles::from_degrees(0.0);

//home
  move_group_arm.setNamedTarget("home");
  move_group_arm.move();

 // ハンドを開く
  gripper_joint_values[0] = GRIPPER_OPEN;
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();


  // TF取得（把持対象）

  geometry_msgs::msg::TransformStamped target_tf;
  try {
    target_tf = tf_buffer.lookupTransform(
      "base_link",
      target_frame,
      tf2::TimePointZero
    );
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(LOGGER, "TF error: %s", ex.what());
    rclcpp::shutdown();
    return 1;
  }

  geometry_msgs::msg::Pose target_pose;
  tf2::Quaternion q;

  // 上から近づ駆動さ
  target_pose.position.x = target_tf.transform.translation.x;
  target_pose.position.y = target_tf.transform.translation.y;
  target_pose.position.z = target_tf.transform.translation.z + 0.10;

  q.setRPY(
    angles::from_degrees(-180),
    angles::from_degrees(-30),
    angles::from_degrees(0)
  );
  target_pose.orientation = tf2::toMsg(q);
  move_group_arm.setPoseTarget(target_pose);
  move_group_arm.move();

  //掴みにいく動作
  target_pose.position.z -= 0.05;
  move_group_arm.setPoseTarget(target_pose);
  move_group_arm.move();

  // 掴む
  gripper_joint_values[0] = GRIPPER_CLOSE;
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();

  //持ち上げ
  target_pose.position.z += 0.15;
  move_group_arm.setPoseTarget(target_pose);
  move_group_arm.move();

  
  // 配置位置（parameter）
  
  target_pose.position.x = place_x;
  target_pose.position.y = place_y;
  target_pose.position.z = place_z + 0.1;
  move_group_arm.setPoseTarget(target_pose);
  move_group_arm.move();

  target_pose.position.z = place_z;
  move_group_arm.setPoseTarget(target_pose);
  move_group_arm.move();

  // 離す
  gripper_joint_values[0] = GRIPPER_OPEN;
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();

  
  // 戻る動作
  move_group_arm.setNamedTarget("home");
  move_group_arm.move();

  gripper_joint_values[0] = GRIPPER_DEFAULT;
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();

  rclcpp::shutdown();
  return 0;
}


  // 掴む準備をする
//  geometry_msgs::msg::Pose target_pose;
//  tf2::Quaternion q;
//  target_pose.position.x = 0.2;
//  target_pose.position.y = 0.0;
//  target_pose.position.z = 0.2;

 
