cmake_minimum_required(VERSION 3.8)
project(lidar_undistortion)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wpedantic -Wextra)

find_package(ament_cmake REQUIRED)
find_package(Eigen3)

find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)
# comment above and uncomment below to enable unit test debugging with gui
# find_package(OpenCV REQUIRED COMPONENTS core imgcodecs highgui)

# Stop build warnings related to missing IO components
set(PCL_FIND_QUIETLY ON)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(jsoncpp REQUIRED)
get_target_property(jsoncpp_INCLUDE_DIRS jsoncpp_lib INTERFACE_INCLUDE_DIRECTORIES)


include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${jsoncpp_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME} src/pose_buffer.cpp)

add_library(ouster_undistortion src/ouster_undistorter.cpp
                                src/ouster_image_converter.cpp
                                src/ouster_metadata_utils.cpp
                                src/ouster_config.cpp)

target_link_libraries(ouster_undistortion ${PROJECT_NAME}
                                          ${catkin_LIBRARIES}
                                          ${OpenCV_LIBRARIES}
                                          jsoncpp_lib)

add_library(velodyne_undistortion src/velodyne_container.cpp
                                  src/velodyne_container_organized.cpp
                                  src/velodyne_rawdata.cpp
                                  src/velodyne_calibration.cpp)

target_link_libraries(velodyne_undistortion ${catkin_LIBRARIES}
                                            ${velodyne_pointcloud_LIBRARIES}
                                            ${OpenCV_LIBRARIES}
                                            ${PCL_LIBRARIES}
                                            yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pose_buffer test/pose_buffer_test.cpp)
  target_link_libraries(test_pose_buffer gtest_main ${PROJECT_NAME})

  ament_add_gtest(test_ouster_undistortion test/fill_clouds.cpp
                                          test/ouster_undistortion_test.cpp)

  target_link_libraries(test_ouster_undistortion gtest_main
                                                 ouster_undistortion
                                                 ${Boost_LIBRARIES}
                                                 ${PCL_LIBRARIES}
                                                 jsoncpp_lib)

  ament_add_gtest(test_ouster_img_converter test/ouster_image_converter_test.cpp)
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

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} ouster_undistortion velodyne_undistortion)
ament_export_dependencies(PCL OpenCV)

ament_package()