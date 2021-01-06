cmake_minimum_required(VERSION 2.8.3)
project(lidar_undistortion)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED)

find_package(Eigen3)

find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)
# comment above and uncomment below to enable unit test debugging with gui
# find_package(OpenCV REQUIRED COMPONENTS core imgcodecs highgui)

find_package(PCL REQUIRED COMPONENTS common io)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME} ouster_undistortion velodyne_undistortion
               CATKIN_DEPENDS ${CATKIN_DEP_TO_ADD}
               DEPENDS PCL OpenCV)



include_directories(include ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})


add_library(${PROJECT_NAME} src/pose_buffer.cpp)

add_library(ouster_undistortion src/ouster_undistorter.cpp
                                src/ouster_image_converter.cpp
                                src/ouster_metadata_utils.cpp
                                src/ouster_config.cpp)

target_link_libraries(ouster_undistortion ${PROJECT_NAME}
                                          ${catkin_LIBRARIES}
                                          ${OpenCV_LIBRARIES})

add_library(velodyne_undistortion src/velodyne_container.cpp
                                  src/velodyne_container_organized.cpp
                                  src/velodyne_rawdata.cpp
                                  src/velodyne_calibration.cpp)

target_link_libraries(velodyne_undistortion ${catkin_LIBRARIES}
                                            ${velodyne_pointcloud_LIBRARIES}
                                            ${OpenCV_LIBRARIES}
                                            ${PCL_LIBRARIES}
                                            yaml-cpp)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_pose_buffer test/pose_buffer_test.cpp)
  target_link_libraries(test_pose_buffer gtest_main ${PROJECT_NAME})

  catkin_add_gtest(test_ouster_undistortion test/fill_clouds.cpp
                                          test/ouster_undistortion_test.cpp)

  target_link_libraries(test_ouster_undistortion gtest_main ouster_undistortion)

  catkin_add_gtest(test_ouster_img_converter test/ouster_image_converter_test.cpp)
  target_link_libraries(test_ouster_img_converter gtest_main
                                                  ouster_undistortion
                                                  ${Boost_LIBRARIES}
                                                  ${OpenCV_LIBRARIES}
                                                  ${PCL_LIBRARIES})
endif()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME} ouster_undistortion velodyne_undistortion
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
