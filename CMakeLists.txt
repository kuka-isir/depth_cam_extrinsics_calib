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

add_executable(tf_to_pt src/tf_to_pointstamped.cpp)
add_dependencies(tf_to_pt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf_to_pt
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
 )

 install(TARGETS hsv_segmentation tf_to_pt
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   PATTERN ".svn" EXCLUDE
 )
