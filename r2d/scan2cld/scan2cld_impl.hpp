#pragma once
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <memory>
#include <Eigen/Eigen>

namespace rclcpp {
class Node;
};

namespace r2d {
class scan2cld_impl {
using transform_t = Eigen::Transform<double, 3, Eigen::Affine>;
public:
  scan2cld_impl(std::shared_ptr<rclcpp::Node>);
  ~scan2cld_impl();
  sensor_msgs::msg::PointCloud2::SharedPtr
  process(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  sensor_msgs::msg::PointCloud2::SharedPtr
  process(const sensor_msgs::msg::LaserScan &msg);

private:
  void load_param();
  transform_t load_transform(std::string name);
  void update_scan_info(const sensor_msgs::msg::LaserScan &msg);

private:
  std::shared_ptr<rclcpp::Node> ndp_;
  transform_t T_lb_lm_{transform_t::Identity()};
  transform_t T_m_w_{transform_t::Identity()};
  transform_t T_inc{transform_t::Identity()};
  Eigen::Vector3d rotation_axis_{Eigen::Vector3d::UnitX()};
  double rotation_speed_{0.0};
  bool is_positive_rotate_{true};
  double time_one_msg_{0.0};
  double time_inc_{0.0};
  double time_blind_{0.0};
  double rad_inc_per_point_{0.0};
};
};