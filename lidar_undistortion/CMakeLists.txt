cmake_minimum_required(VERSION 2.8.3)
project(lidar_undistortion)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS std_msgs
                                        cv_bridge
                                        message_generation
                                        image_transport)

find_package(Eigen3)
find_package(OpenCV REQUIRED COMPONENTS core)
find_package(PCL REQUIRED COMPONENTS common io)

list(APPEND LIBRARIES_TO_BUILD "lidar_undistortion")
list(APPEND CATKIN_DEP_TO_ADD "message_runtime")
set(EXECUTABLES_TO_BUILD "")

find_package(ouster_ros QUIET)
if(${ouster_ros_FOUND})
  list(APPEND LIBRARIES_TO_BUILD "ouster_undistortion")
  list(APPEND CATKIN_DEP_TO_ADD ouster_ros)
else()
  message(WARNING "package ouster_ros not found. Not building ouster_undistortion.")
endif()

find_package(velodyne_pointcloud QUIET)
if(${velodyne_pointcloud_FOUND})
  message(STATUS "Package velodyne_pointcloud found. Building velodyne_undistorter.")
  list(APPEND LIBRARIES_TO_BUILD "velodyne_undistortion")
else()
  message(WARNING "package velodyne_pointcloud not found. Not building velodyne_undistortion.")
endif()

add_message_files(FILES RangeImage.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(INCLUDE_DIRS include
               LIBRARIES ${LIBRARIES_TO_BUILD}
               CATKIN_DEPENDS ${CATKIN_DEP_TO_ADD})



include_directories(include ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})


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
                                          src/ouster_undistorter_ros.cpp)

  target_link_libraries(ouster_undistortion_node ouster_undistortion
                                                 ${catkin_LIBRARIES})

  add_dependencies(ouster_undistortion_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



  add_executable(ouster_image_converter_node src/ouster_image_converter_node.cpp)
  target_link_libraries(ouster_image_converter_node ouster_undistortion ${catkin_LIBRARIES})
endif()

if(${velodyne_pointcloud_FOUND})
  include_directories(${velodyne_pointcloud_INCLUDE_DIRS})
  add_library(velodyne_undistortion src/velodyne_container.cpp
                                    src/velodyne_container_organized.cpp
                                    src/velodyne_image_converter.cpp
                                    src/velodyne_rawdata.cpp)

  target_link_libraries(velodyne_undistortion lidar_undistortion
                                              ${catkin_LIBRARIES}
                                              ${velodyne_pointcloud_LIBRARIES}
                                              ${OpenCV_LIBRARIES}
                                              ${PCL_LIBRARIES})

  add_executable(velodyne_undistortion_node src/velodyne_undistortion_node.cpp
                                            src/velodyne_undistorter_ros.cpp)

  target_link_libraries(velodyne_undistortion_node velodyne_undistortion
                                                   ${catkin_LIBRARIES})

  add_executable(velodyne_image_converter_node src/velodyne_image_converter_node.cpp)
  target_link_libraries(velodyne_image_converter_node velodyne_undistortion
                                                      ${velodyne_pointcloud_LIBRARIES}
                                                      ${catkin_LIBRARIES})
  
endif()


catkin_add_gtest(test_pose_buffer test/pose_buffer_test.cpp)
target_link_libraries(test_pose_buffer gtest_main lidar_undistortion)

catkin_add_gtest(test_ouster_undistortion test/fill_clouds.cpp
                                          test/ouster_undistortion_test.cpp)

target_link_libraries(test_ouster_undistortion gtest_main ouster_undistortion)


