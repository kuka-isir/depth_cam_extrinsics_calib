cmake_minimum_required(VERSION 2.8.3)
project(depth_cam_extrinsics_calib)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  openni_camera
  openni_launch
  depth_cam_tools
  rospy
  geometry_msgs
  message_generation
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  visualization_msgs
  std_srvs
  dynamic_reconfigure
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/params.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(hsv_segmentation src/hsv_segmentation.cpp)
add_dependencies(hsv_segmentation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hsv_segmentation
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
 )
