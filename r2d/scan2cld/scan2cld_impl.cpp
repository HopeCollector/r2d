#include "r2d/scan2cld/scan2cld_impl.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>

using namespace r2d;

scan2cld_impl::scan2cld_impl(std::shared_ptr<rclcpp::Node> node) : ndp_(node) {
  load_param();
}

scan2cld_impl::~scan2cld_impl() {}

sensor_msgs::msg::PointCloud2::SharedPtr
scan2cld_impl::process(sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
  return process(*msg);
}

sensor_msgs::msg::PointCloud2::SharedPtr
scan2cld_impl::process(const sensor_msgs::msg::LaserScan &msg) {
  update_scan_info(msg);
  auto &ranges = msg.ranges;
  auto rad_local_point = msg.angle_min;
  auto rad_motor = msg.intensities[0];
  transform_t T_lm_m{transform_t::Identity()};
  T_lm_m.linear() =
      Eigen::AngleAxisd(rad_motor, rotation_axis_).toRotationMatrix();
  Eigen::Vector3d p_local, p_world;
  p_local[2] = 0.0;
  pcl::PointCloud<pcl::PointXYZ> cld;
  cld.resize(ranges.size());
  for (int i = 0; i < ranges.size(); i++) {
    const auto &range = ranges[i];
    auto &p = cld[i];
    if (range < msg.range_min || range > msg.angle_max)
      continue;
    p_local[0] = range * cos(rad_local_point);
    p_local[1] = range * sin(rad_local_point);
    p_world = T_m_w_ * T_lm_m * T_lb_lm_ * p_local;
    p.x = p_world[0];
    p.y = p_world[1];
    p.z = p_world[2];
    T_lm_m = T_inc * T_lm_m;
    rad_local_point += msg.angle_increment;
  }

  sensor_msgs::msg::PointCloud2::SharedPtr ret(
      new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(cld, *ret);
  ret->header = msg.header;
  return ret;
}

void scan2cld_impl::load_param() {
  T_lb_lm_ = load_transform("T_lb_lm");
  T_m_w_ = load_transform("T_m_w");
  ndp_->declare_parameter("r2d.scan2cld.rotation_axis",
                          std::vector<double>{1, 0, 0});
  ndp_->declare_parameter("r2d.scan2cld.rotate_speed", 0.0);
  ndp_->declare_parameter("r2d.scan2cld.is_positive_rotate", true);
  std::vector<double> axis;
  ndp_->get_parameter("r2d.scan2cld.rotation_axis", axis);
  rotation_speed_ =
      ndp_->get_parameter("r2d.scan2cld.rotate_speed").as_double();
  is_positive_rotate_ =
      ndp_->get_parameter("r2d.scan2cld.is_positive_rotate").as_bool();
  rotation_axis_ << axis[0], axis[1], axis[2];
}

scan2cld_impl::transform_t scan2cld_impl::load_transform(std::string name) {
  Eigen::Matrix4d mat;
  name = "r2d.scan2cld." + name + ".";
  for (int i = 0; i < 4; i++) {
    ndp_->declare_parameter(name + "row" + std::to_string(i + 1),
                            std::vector<double>{});
    const auto &row = ndp_->get_parameter(name + "row" + std::to_string(i + 1))
                          .as_double_array();
    mat.row(i) << row[0], row[1], row[2], row[3];
  }
  return transform_t{mat};
}

void scan2cld_impl::update_scan_info(
    const sensor_msgs::msg::LaserScan &msg) {
  static bool is_updated_ = false;
  if (is_updated_)
    return;
  is_updated_ = true;
  time_one_msg_ = msg.scan_time;
  time_inc_ = msg.time_increment;
  time_blind_ = time_one_msg_ - msg.intensities.size() * time_inc_;
  rad_inc_per_point_ =
      (is_positive_rotate_ ? 1 : -1) * rotation_speed_ * time_inc_;
  T_inc.linear() =
      Eigen::AngleAxisd(rad_inc_per_point_, rotation_axis_).toRotationMatrix();
}