cmake_minimum_required(VERSION 2.8.3)
project(lidar_undistortion)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS pcl_conversions)

find_package(Eigen3)
find_package(OpenCV REQUIRED COMPONENTS core)
find_package(PCL REQUIRED COMPONENTS common io)

list(APPEND LIBRARIES_TO_BUILD ${PROJECT_NAME})
set(EXECUTABLES_TO_BUILD "")

find_package(ouster_client REQUIRED)
if(${ouster_client_FOUND})
  list(APPEND LIBRARIES_TO_BUILD "ouster_undistortion")
  list(APPEND CATKIN_DEP_TO_ADD ouster_client)
else()
  message(WARNING "package ouster_client not found. Not building ouster_undistortion.")
endif()



catkin_package(INCLUDE_DIRS include
               LIBRARIES ${LIBRARIES_TO_BUILD} velodyne_undistortion
               CATKIN_DEPENDS ${CATKIN_DEP_TO_ADD}
               DEPENDS PCL OpenCV)



include_directories(include ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})


add_library(${PROJECT_NAME} src/pose_buffer.cpp)

if(${ouster_client_FOUND})
  message(STATUS "Package ouster_ros found. Building ouster_undistorter.")
  include_directories(${ouster_client_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
  add_library(ouster_undistortion src/ouster_undistorter.cpp
                                  src/ouster_image_converter.cpp)
  target_link_libraries(ouster_undistortion ${PROJECT_NAME}
                                            ${catkin_LIBRARIES}
                                            ${ouster_client_LIBRARIES}
                                            ${OpenCV_LIBRARIES})


endif()

add_library(velodyne_undistortion src/velodyne_container.cpp
                                  src/velodyne_container_organized.cpp
                                  src/velodyne_rawdata.cpp
                                  src/velodyne_calibration.cpp)

target_link_libraries(velodyne_undistortion ${catkin_LIBRARIES}
                                            ${velodyne_pointcloud_LIBRARIES}
                                            ${OpenCV_LIBRARIES}
                                            ${PCL_LIBRARIES}
                                            yaml-cpp)




add_executable(test_ouster_img_converter test/ouster_image_converter_test.cpp)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_pose_buffer test/pose_buffer_test.cpp)
  target_link_libraries(test_pose_buffer gtest_main ${PROJECT_NAME})

  catkin_add_gtest(test_ouster_undistortion test/fill_clouds.cpp
                                          test/ouster_undistortion_test.cpp)

  target_link_libraries(test_ouster_undistortion gtest_main ouster_undistortion)
endif()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${LIBRARIES_TO_BUILD}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
