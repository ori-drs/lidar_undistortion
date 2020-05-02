cmake_minimum_required(VERSION 2.8.3)
project(lidar_undistortion)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(lidar_undistortion src/lidar_undistorter.cpp
                                  src/pose_buffer.cpp)

cs_add_executable(lidar_undistortion_node
    src/lidar_undistortion_node.cpp
    src/lidar_undistorter_ros.cpp)

target_link_libraries(lidar_undistortion_node
                      lidar_undistortion)


cs_add_executable(pose_buffer_test src/pose_buffer.cpp
                                   src/pose_buffer_test.cpp)

cs_add_executable(lidar_undistortion_test src/lidar_undistortion_test.cpp
                                          src/fill_clouds.cpp)

target_link_libraries(lidar_undistortion_test lidar_undistortion)
