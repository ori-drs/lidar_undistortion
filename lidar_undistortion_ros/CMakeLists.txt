cmake_minimum_required(VERSION 3.0.2)
project(lidar_undistortion_ros)

# Some of the code features require C++14
SET(CMAKE_CXX_STANDARD 14)

add_compile_options(-Wall -Wpedantic -Wextra)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp image_transport cv_bridge tf2_ros lidar_undistortion lidar_undistortion_msgs ouster_ros velodyne_msgs)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})


list(APPEND LIBRARIES_TO_BUILD ${PROJECT_NAME})


list(APPEND VELODYNE_TARGETS "velodyne_undistortion_ros")
list(APPEND VELODYNE_TARGETS "velodyne_undistortion_node")
list(APPEND VELODYNE_TARGETS "velodyne_image_converter_node")


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${LIBRARIES_TO_BUILD}
  CATKIN_DEPENDS roscpp ouster_ros lidar_undistortion_msgs velodyne_msgs
  DEPENDS EIGEN3)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library

include_directories(${velodyne_pointcloud_INCLUDE_DIRS})
add_library(velodyne_undistortion_ros src/velodyne_image_converter.cpp
                                      src/velodyne_scan_converter.cpp
                                      src/velodyne_undistorter_ros.cpp)
 
target_link_libraries(velodyne_undistortion_ros ${catkin_LIBRARIES}
                                                ${velodyne_pointcloud_LIBRARIES}
                                                ${OpenCV_LIBRARIES}
                                                ${PCL_LIBRARIES})

add_executable(velodyne_undistortion_node src/velodyne_undistortion_node.cpp)

target_link_libraries(velodyne_undistortion_node velodyne_undistortion_ros
                                                 ${catkin_LIBRARIES})

add_executable(velodyne_image_converter_node src/velodyne_image_converter_node.cpp)
target_link_libraries(velodyne_image_converter_node velodyne_undistortion_ros
                                                    ${velodyne_pointcloud_LIBRARIES}
                                                    ${catkin_LIBRARIES})

add_library(${PROJECT_NAME} src/ouster_undistorter_ros.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(ouster_undistortion_node src/ouster_undistortion_node.cpp)
target_link_libraries(ouster_undistortion_node ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(ouster_undistortion_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



add_executable(ouster_image_converter_node src/ouster_image_converter_node.cpp)
target_link_libraries(ouster_image_converter_node ${catkin_LIBRARIES})


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
install(TARGETS ouster_undistortion_node ouster_image_converter_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME} ${VELODYNE_TARGETS}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp")

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
