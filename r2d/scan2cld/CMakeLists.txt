ament_auto_add_library(scan2cld SHARED scan2cld.cpp scan2cld_impl.cpp)
target_include_directories(scan2cld 
  PRIVATE ${PRIVATE_COMMON_INCLUDE_DIR}
          ${PCL_INCLUDE_DIRS}
          ${rclcpp_INCLUDE_DIRS}
          ${sensor_msgs_INCLUDE_DIRS}
          ${pcl_conversions_INCLUDE_DIRS}
  PUBLIC  $<BUILD_INTERFACE:${PUBLIC_COMMON_INCLUDE_DIR}>
          $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(scan2cld 
  PRIVATE ${PCL_LIBRARIES}
          ${rclcpp_LIBRARIES}
          ${sensor_msgs_LIBRARIES}
          ${pcl_conversions_LIBRARIES}
)
install(TARGETS scan2cld
  EXPORT ${PROJECT_NAME}-config
  RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
)