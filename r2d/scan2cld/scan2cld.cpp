#include "r2d/scan2cld.hpp"
#include "r2d/scan2cld/scan2cld_impl.hpp"
#include <rclcpp/rclcpp.hpp>

using namespace r2d;

scan2cld::scan2cld(std::shared_ptr<rclcpp::Node> ndp)
    : impl_(new scan2cld_impl{ndp}) {}

scan2cld::~scan2cld() {}

sensor_msgs::msg::PointCloud2::SharedPtr
scan2cld::process(sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
  return impl_->process(msg);
}

sensor_msgs::msg::PointCloud2::SharedPtr
scan2cld::process(const sensor_msgs::msg::LaserScan &msg) {
  return impl_->process(msg);
}