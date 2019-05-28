# 2019-05-24 16:54:54

cmake_minimum_required(VERSION 2.8.3)
project(robot_assist)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_assist
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs tf
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)


add_executable(dxl_imu src/dxl_imu.cpp)
target_link_libraries(dxl_imu LINK_PUBLIC "/usr/local/lib/libdxl_x64_cpp.so")   #for Linux
# target_link_libraries(dxl_imu LINK_PUBLIC "/usr/local/lib/libdxl_sbc_cpp.so")     #for Jetson
target_link_libraries(dxl_imu ${catkin_LIBRARIES})

add_executable(imu_sub src/imu_sub.cpp)
target_link_libraries(imu_sub ${catkin_LIBRARIES})

add_executable(assist_dual src/assist_dual.cpp)
target_link_libraries(assist_dual LINK_PUBLIC "/usr/local/lib/libdxl_x64_cpp.so")
# target_link_libraries(assist_dual LINK_PUBLIC "/usr/local/lib/libdxl_sbc_cpp.so")
target_link_libraries(assist_dual ${catkin_LIBRARIES})

add_executable(dxl_torq src/dxl_torq.cpp)
target_link_libraries(dxl_torq LINK_PUBLIC "/usr/local/lib/libdxl_x64_cpp.so")
# target_link_libraries(dxl_torq LINK_PUBLIC "/usr/local/lib/libdxl_sbc_cpp.so")
target_link_libraries(dxl_torq ${catkin_LIBRARIES})
