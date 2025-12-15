// Copyright 2025 
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

#include <cmath>
#include <iostream>
#include <memory>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_geometry/pinhole_camera_model.h"

using std::placeholders::_1;

class ColorDetectionNode : public rclcpp::Node
{
public:
  ColorDetectionNode()
  : Node("color_detection")
  {
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/color/image_raw", 10,
      std::bind(&ColorDetectionNode::image_callback, this, _1));

    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/aligned_depth_to_color/image_raw", 10,
      std::bind(&ColorDetectionNode::depth_callback, this, _1));

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/color/camera_info", 10,
      std::bind(&ColorDetectionNode::camera_info_callback, this, _1));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(), "Color detection node started");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
  sensor_msgs::msg::Image::SharedPtr depth_image_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  struct HSVRange
  {
    int low_h, high_h;
    int low_s, high_s;
    int low_v, high_v;
  };

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!camera_info_ || !depth_image_) {
      return;
    }


// (勉強メモ)
//   struct HSVRange
//  {
//    int low_h, high_h;
//    int low_s, high_s;
//    int low_v, high_v;
//  };
//で定義したように、low_h = 100, high_h = 130, low_s = 100, high_s = 255, low_v = 50, high_v = 255をまとめて書く
    // ===== 色ごとのHSV範囲 =====
    std::map<std::string, HSVRange> color_ranges = {
//色相Hue: 100〜130 → 青, 彩度Saturation: 100〜255 → 鮮やか, 明度Value: 50〜255 → 暗すぎない
      { "blue_cube",   {100, 130, 100, 255,  50, 255} },
      { "yellow_cube", { 20,  35, 100, 255,  30, 255} },
      // 赤は HSV が 0 を跨ぐので2回処理する(赤は0付近と179付近に分かれて存在する。まずは片側だけ処理)
      { "red_cube",    {  0,  10, 100, 255,  50, 255} }
    };

    auto cv_img = cv_bridge::toCvShare(msg, msg->encoding);
    cv::Mat hsv_image;
    cv::cvtColor(cv_img->image, hsv_image, cv::COLOR_RGB2HSV);

    image_geometry::PinholeCameraModel camera_model;
    camera_model.fromCameraInfo(camera_info_);

    auto cv_depth = cv_bridge::toCvShare(depth_image_, depth_image_->encoding);

    for (const auto &pair : color_ranges) {
      const std::string &frame_name = pair.first;
      const HSVRange &hsv = pair.second;

      cv::Mat mask;

      if (frame_name == "red_cube") {
        // 赤は 0-10 と 170-180 を合成
        cv::Mat mask1, mask2;
        cv::inRange(
          hsv_image,
          cv::Scalar(0, hsv.low_s, hsv.low_v),
          cv::Scalar(10, hsv.high_s, hsv.high_v),
          mask1);
        cv::inRange(
          hsv_image,
          cv::Scalar(170, hsv.low_s, hsv.low_v),
          cv::Scalar(180, hsv.high_s, hsv.high_v),
          mask2);
        mask = mask1 | mask2;
      } else {
        cv::inRange(
          hsv_image,
          cv::Scalar(hsv.low_h, hsv.low_s, hsv.low_v),
          cv::Scalar(hsv.high_h, hsv.high_s, hsv.high_v),
          mask);
      }

      // ノイズ除去
      cv::morphologyEx(
        mask, mask, cv::MORPH_OPEN,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
      cv::morphologyEx(
        mask, mask, cv::MORPH_CLOSE,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

      cv::Moments m = moments(mask);
      if (m.m00 < 10000) {
        continue;
      }

      const double pixel_x = m.m10 / m.m00;
      const double pixel_y = m.m01 / m.m00;

      cv::Point2d pixel(pixel_x, pixel_y);
      cv::Point2d rect_pixel = camera_model.rectifyPoint(pixel);
      cv::Point3d ray = camera_model.projectPixelTo3dRay(rect_pixel);

      const double DEPTH_OFFSET = 0.015;
      const double DEPTH_MIN = 0.2;
      const double DEPTH_MAX = 0.5;

      double front_distance =
        cv_depth->image.at<uint16_t>(pixel) / 1000.0;
      double center_distance = front_distance + DEPTH_OFFSET;

      if (center_distance < DEPTH_MIN || center_distance > DEPTH_MAX) {
        continue;
      }

      cv::Point3d pos(
        ray.x * center_distance,
        ray.y * center_distance,
        ray.z * center_distance);

      geometry_msgs::msg::TransformStamped t;
      t.header = msg->header;
      t.child_frame_id = frame_name;
      t.transform.translation.x = pos.x;
      t.transform.translation.y = pos.y;
      t.transform.translation.z = pos.z;

      tf_broadcaster_->sendTransform(t);
    }
  }

  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    camera_info_ = msg;
  }

  void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    depth_image_ = msg;
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ColorDetectionNode>());
  rclcpp::shutdown();
  return 0;
}

