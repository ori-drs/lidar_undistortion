# lidar_undistortion
This package contains libraries and ROS nodes that perform the following operations:
- Motion compensation of point clouds from a sequence of poses from an external source
- Conversion of point clouds into spherical coordinates (range, azimuth and altitude) and viceversa
- Conversion of range data into regular single channel images, for visualization purposes
- Pose buffering with interpolation

## History
The code in this repo was originally developed and open sourced by the ASL group of ETH Zurich [in this repo](https://github.com/ethz-asl/lidar_undistortion) in 2020.

Between 2020-2023 members of Oxford Robotics Institute added extra features to support the Hesai lidar and to fix bugs. In 2025 ORI ported the code to ROS2 and open sourced this particular repo.

## Supported devices
- Hesai XT32 and QT64
- Ouster OS1
- Velodyne VLP-16 (range conversion partially working)

## ROS nodes
### `ouster_undistortion_node`
This node subscribes to lidar point clouds in lidar frame and sensor poses in a fixed frame as `sensor_msgs::PointCloud2` and `geometry_msgs::PoseWithCovarianceStamped` messages, respectively. Note that the covariance of the message is not used at the moment.
The node outputs a motion compensated point cloud in lidar frame. Both input and output clouds are wrapped around the [point definition](https://github.com/ouster-lidar/ouster_example/blob/master/ouster_ros/include/ouster_ros/point_os1.h) by Ouster, which includes the following fields: `x, y, z, intensity, t, reflectivity, ring, noise, range`. See the official manual for more information

### `ouster_image_converter_node`
This node subscribes to an Ouster point cloud as `sensor_msgs::PointCloud2` and converts them into range, altitude and azimuth values (a.k.a. spherical coordinates). These values are published on ROS as a [custom message](https://github.com/ori-drs/lidar_undistortion/blob/master/lidar_undistortion/msg/RangeImage.msg) of type `lidar_undistortion::RangeImage`. It also converts the spherical coordinates, intensity, and reflectivity values into single channel images and publish them.

### `velodyne_undistortion_node`
Equivalent node for the Velodyne VLP-16 (a.k.a. Velodyne Puck). It works but has not been tested as extensively as the Ouster counterpart.

### `velodyne_image_converter_node`
Equivalent node for the Velodyne VLP-16 (a.k.a. Velodyne Puck). It runs but the image conversion is still work in progress (the image is upside down).

## Dependencies
- `ouster_ros`: can be cloned from [here](https://github.com/ouster-lidar/ouster_example)
- `velodyne_pointcloud`: can be installed from `apt` via `sudo apt install ros-melodic-velodyne-pointcloud`

Both dependencies are optional. However, if neither of these are found on the system, only a library with
minimal capabilities is built.

## Target System
Tested only on `Ubuntu 18.04` and `ROS Melodic`
In particular, the version for `velodyne_pointcloud` is the latest release for Melodic, which corresponds to version 1.5.2

## How to Run
To run the undistortion nodes, look at the `os1_undistortion.launch` and `vlp16_undistortion.launch` launch files, edit the appropriate fields for the reference frames and topics and run them as usual.

To run the image converter node, just run it with no paramenters:
```
rosrun lidar_undistortion ouster_image_converter_node
```
