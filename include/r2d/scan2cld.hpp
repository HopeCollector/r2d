#pragma once
#include <memory>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace rclcpp {
  class Node;
};

namespace r2d {
class scan2cld_impl;

class scan2cld {
public:
  scan2cld(std::shared_ptr<rclcpp::Node>);
  ~scan2cld();
  sensor_msgs::msg::PointCloud2::SharedPtr
  process(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  sensor_msgs::msg::PointCloud2::SharedPtr
  process(const sensor_msgs::msg::LaserScan &msg);

private:
  std::unique_ptr<scan2cld_impl> impl_;
};
};