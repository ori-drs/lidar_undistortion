cmake_minimum_required(VERSION 3.8)
project(lidar_undistortion_ros)

# Some of the code features require C++14
SET(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find catkin macros and libraries
find_package(ament_cmake REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(lidar_undistortion REQUIRED)
find_package(lidar_undistortion_msgs REQUIRED)
find_package(velodyne_msgs REQUIRED)

find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(yaml-cpp REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
include_directories(include 
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


list(APPEND LIBRARIES_TO_BUILD ${PROJECT_NAME})


list(APPEND VELODYNE_TARGETS "velodyne_undistortion_ros")
list(APPEND VELODYNE_TARGETS "velodyne_undistortion_node")
list(APPEND VELODYNE_TARGETS "velodyne_image_converter_node")


###########
## Build ##
###########

## Declare a C++ library

include_directories(${velodyne_pointcloud_INCLUDE_DIRS})
add_library(velodyne_undistortion_ros src/velodyne_image_converter.cpp
                                      src/velodyne_scan_converter.cpp
                                      src/velodyne_undistorter_ros.cpp)

ament_target_dependencies(velodyne_undistortion_ros
  lidar_undistortion
  lidar_undistortion_msgs
  velodyne_msgs
  image_transport
  pcl_conversions
  cv_bridge
  tf2_ros
  tf2_eigen
  yaml-cpp
)
target_link_libraries(velodyne_undistortion_ros yaml-cpp)


add_executable(velodyne_undistortion_node src/velodyne_undistortion_node.cpp)

target_link_libraries(velodyne_undistortion_node velodyne_undistortion_ros)

add_executable(velodyne_image_converter_node src/velodyne_image_converter_node.cpp)
ament_target_dependencies(velodyne_image_converter_node
  image_transport
  cv_bridge
)
target_link_libraries(velodyne_image_converter_node velodyne_undistortion_ros
                                                    ${velodyne_pointcloud_LIBRARIES})

add_library(${PROJECT_NAME} # src/ouster_undistorter_ros.cpp
                            src/hesai_undistorter_ros.cpp)
ament_target_dependencies(${PROJECT_NAME}
  lidar_undistortion
  lidar_undistortion_msgs
  image_transport
  cv_bridge
  tf2_ros
  tf2_eigen
  pcl_conversions
)

# add_executable(ouster_undistortion_node src/ouster_undistortion_node.cpp)
# target_link_libraries(ouster_undistortion_node ${PROJECT_NAME} ${catkin_LIBRARIES})
# add_dependencies(ouster_undistortion_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(hesai_undistortion_node src/hesai_undistortion_node.cpp)
ament_target_dependencies(hesai_undistortion_node
  lidar_undistortion
  lidar_undistortion_msgs
  image_transport
  cv_bridge
  tf2_ros
  tf2_eigen
  pcl_conversions
)
target_link_libraries(hesai_undistortion_node ${PROJECT_NAME})

# add_executable(ouster_image_converter_node src/ouster_image_converter_node.cpp)
# target_link_libraries(ouster_image_converter_node ${catkin_LIBRARIES})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS
  hesai_undistortion_node
  # ouster_image_converter_node
  # ouster_undistortion_node
  velodyne_image_converter_node
  velodyne_undistortion_node
 DESTINATION ${PROJECT_NAME})

## Mark cpp header files for installation
install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.hpp")

ament_export_include_directories(include)
ament_export_dependencies(cv_bridge image_transport lidar_undistortion velodyne_msgs)
ament_package()
## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_lidar_undistortion_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
