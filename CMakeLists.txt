cmake_minimum_required(VERSION 3.3)
project(r2d)
cmake_policy(SET CMP0057 NEW)
include(GNUInstallDirs)

find_package(PCL COMPONENTS common REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

set(PRIVATE_COMMON_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(PUBLIC_COMMON_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_subdirectory(r2d)

install(EXPORT ${PROJECT_NAME}-config
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

install(DIRECTORY cfg_private/
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cfg
)
ament_auto_package()