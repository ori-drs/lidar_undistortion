cmake_minimum_required(VERSION 2.8.3)
project(lidar_undistortion)

add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS std_msgs
  cv_bridge message_generation image_transport)

find_package(Eigen3)
find_package(OpenCV REQUIRED COMPONENTS core)

set(LIBRARIES_TO_BUILD "lidar_undistortion")
set(EXECUTABLES_TO_BUILD "")

find_package(ouster_ros QUIET)
if(${ouster_ros_FOUND})
  set(LIBRARIES_TO_BUILD "${BUILD_LIBRARIES} ouster_undistortion")
else()
  message(WARNING "package ouster_ros not found. Not building ouster_undistortion.")
endif()

find_package(velodyne_pointcloud QUIET)
if(${velodyne_pointcloud_FOUND})
  message(STATUS "Package velodyne_pointcloud found. Building velodyne_undistorter.")
  set(BUILD_LIBRARIES "${BUILD_LIBRARIES} velodyne_undistortion")
else()
  message(WARNING "package velodyne_pointcloud not found. Not building velodyne_undistortion.")
endif()

add_message_files(FILES RangeImage.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(LIBRARIES ${LIBRARIES_TO_BUILD}
               CATKIN_DEPENDS message_runtime)



include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_library(lidar_undistortion src/pose_buffer.cpp)

if(${ouster_ros_FOUND})
  message(STATUS "Package ouster_ros found. Building ouster_undistorter.")
  include_directories(${ouster_ros_INCLUDE_DIRS})
  add_library(ouster_undistortion src/ouster_undistorter.cpp
                                  src/ouster_image_converter.cpp)
  target_link_libraries(ouster_undistortion lidar_undistortion
                                            ${catkin_LIBRARIES}
                                            ${ouster_ros_LIBRARIES}
                                            ${OpenCV_LIBRARIES})

  add_executable(ouster_undistortion_node src/ouster_undistortion_node.cpp
                                          src/ouster_undistorter_ros.cpp
                                          )

  target_link_libraries(ouster_undistortion_node ouster_undistortion
                                                 ${catkin_LIBRARIES})

  add_dependencies(ouster_undistortion_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

  add_executable(ouster_undistortion_test src/ouster_undistortion_test.cpp
                                          src/fill_clouds.cpp)

  target_link_libraries(ouster_undistortion_test ouster_undistortion)
endif()

if(${velodyne_pointcloud_FOUND})
  include_directories(${velodyne_pointcloud_INCLUDE_DIRS})
  add_library(velodyne_undistortion src/pointcloud_xyzir.cpp)
                                    #src/organized_pointcloud_xyzir.cpp)
  target_link_libraries(velodyne_undistortion lidar_undistortion
                                              ${catkin_LIBRARIES}
                                              ${velodyne_pointcloud_LIBRARIES}
                                              ${OpenCV_LIBRARIES})

  add_executable(velodyne_undistortion_node src/velodyne_undistortion_node.cpp
                                            src/velodyne_undistorter_ros.cpp)
  target_link_libraries(velodyne_undistortion_node velodyne_undistortion
                                                   ${catkin_LIBRARIES})
endif()

add_executable(pose_buffer_test src/pose_buffer.cpp
                                src/pose_buffer_test.cpp)



