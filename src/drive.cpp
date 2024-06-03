/*
 *  SLLIDAR ROS2 CLIENT
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2022 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <math.h>
#include "opencv2/opencv.hpp"

#define RAD2DEG(x) ((x)*180./M_PI)

const int def_speed = 100;
const float k = 70.f;
const int horizontalWallDetectionScore = 700;

float calc_point(float distance) {
    if (distance == 0) return 10.0;
    return 10.0 / distance;
}

static void scanCb(rclcpp::Node::SharedPtr node, sensor_msgs::msg::LaserScan::SharedPtr scan) {
  static auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
  static auto mypub = node->create_publisher<geometry_msgs::msg::Vector3>("/topic_dxlpub", qos_profile);

  int count = scan->scan_time / scan->time_increment;
  printf("[SLLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
  printf("[SLLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));

  cv::Mat img(cv::Size(500, 500), CV_8UC3, cv::Scalar(0, 0, 0));
  //cv::drawMarker(img, cv::Point(250, 250), cv::Scalar(255, 255, 255), cv::MARKER_CROSS, 10, 1, cv::LINE_4);

  cv::drawMarker(img, cv::Point(250, 250), cv::Scalar(0, 100, 0), cv::MARKER_CROSS, 500, 1, cv::LINE_4);
  cv::drawMarker(img, cv::Point(250, 250), cv::Scalar(0, 100, 0), cv::MARKER_STAR, 500, 1, cv::LINE_4);
  for (float r = 50; r <= 250; r += 50) cv::circle(img, cv::Point(250, 250), r, cv::Scalar(0, 100, 0), 1);

  float scoreFrontLeft = 0;
  float scoreFrontRight = 0;

  for (int i = 0; i < count; i++) {
    float angle = scan->angle_min + scan->angle_increment * i;
    float degree = RAD2DEG(angle);
    float distance = scan->ranges[i];

    if (std::isnan(distance) || std::isinf(distance) || distance >= 3) continue;
    int x = 250 + (distance * 100.0 * sin(angle));
    int y = 250 + (distance * 100.0 * cos(angle));

    cv::drawMarker(img, cv::Point(x, y), cv::Scalar(0, 255, 0), cv::MARKER_SQUARE, 2, 2, cv::LINE_4);

    if (degree >= -180 && degree < -120 /* -90 */ && distance < 1.5) {
      scoreFrontLeft += calc_point(distance);
    } else if (degree >= 120 /* 90 */ && degree < 180 && distance < 1.5) {
      scoreFrontRight += calc_point(distance);
    }

    int color = distance * 100;
    if (color >= 255) color = 255;
    cv::drawMarker(img, cv::Point(x, y), cv::Scalar(color, 255, color), cv::MARKER_SQUARE, 2, 2, cv::LINE_4);

    //printf("[SLLIDAR INFO]: angle-distance : [%f, %f]\n", degree, distance);
  }

  float add = abs(scoreFrontLeft - scoreFrontRight) / k;
  printf("절댓값: %f\n", add);

  if (add < 7 && scoreFrontLeft >= horizontalWallDetectionScore && scoreFrontRight >= horizontalWallDetectionScore) {
    printf("벽 감지!\n");
    add = 50;
  }

  if (scoreFrontLeft > scoreFrontRight){
    printf("좌가 점수많음 %f %f\n", scoreFrontLeft, scoreFrontRight);
  } 
  else if (scoreFrontLeft <= scoreFrontRight) {
    printf("우가 점수많음 %f %f\n", scoreFrontLeft, scoreFrontRight);
    add = -1 * add;
  } 

  geometry_msgs::msg::Vector3 speed;

  float vel1 = def_speed + add;
  float vel2 = -1 * (def_speed - add);

  speed.x = vel1;
  speed.y = vel2;

  RCLCPP_INFO(node->get_logger(), "Publish: %lf, %lf", speed.x,  speed.y);
  mypub->publish(speed);
  
  cv::imshow("scan", img);
  cv::waitKey(1);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("lidardrive");

  std::function<void(const sensor_msgs::msg::LaserScan::SharedPtr msg)> fn;
  
  fn = std::bind(scanCb, node, std::placeholders::_1);
  
  auto lidar_info_sub = node->create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS(), fn);

  

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
