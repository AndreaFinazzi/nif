# ROS2 Luminar Drivers

This repository contains an implementation of ROS2 drivers for the Luminar H3 lidar.

The driver is based on the [ROS 2 drivers for the Ouster lidar](https://github.com/ros-drivers/ros2_ouster_drivers).

In order to build the driver you must include the LuminLib-all-2.16.0 SDK package in your ROS 2 workspace, this is included in this repository under `./LuminLib-all-2.16.0` to include the needed header files and precompiled static libraries in both Debug and Release (optimized) configurations.  

## ROS Interfaces

| Topic                | Type                    | Description                                      |
|----------------------|-------------------------|--------------------------------------------------|
|`<params:lidar_topic_name>`|`sensor_msgs::msg::PointCloud2`|3D Pointcloud generated from a 360 rotation. The name of this Topic depends on parameter `lidar_topic_name`.|
|`<params:lidar_topic_name>/flat_zc`|`ros2::flat_zc::sensor_msgs::msg::PointCloud2`|3D Pointcloud generated from a 360 rotation. The name of this Topic depends on parameter `lidar_topic_name`.|

| Service           | Type                    | Description                       |
|-------------------|-------------------------|-----------------------------------|
| `reset`           | std_srvs/Empty          | Reset the sensor's connection     |
| `GetMetadata`     | luminar_msgs/GetMetadata | Get information about the sensor  |

| `ros2_luminar` Parameter | Type    | Description                                                                                                 |
|--------------------------|---------|-------------------------------------------------------------------------------------------------------------|
|`lidar_fingerprint`       |int      |The fingerprint ID of the sensor head to connect to. |
|`data_source_is_pcap_app` |bool     |If True, expect sensor data to come from PCAP player app running on localhost. Use physical sensor otherwise.|
|`scan_rate_hz`            |float    |Full-field scan rate (1 to 30 Hz)|
|`scan_fov`                |float[2] |Scanning vertical field of view in degrees (range(<=30), center(-15 to 15), range+center cannot exceed +/-15_|
|`scan_pattern`            |string[4]|Accepted values: Gaussian, Trapezoidal, Uniform, Horizon, Exponential|
|`coordinate_type`         |string   |"Cartesian" or "Spherical" (Spherical has lower latency)|
|`lidar_topic_name`         |string   |Base topic name for this lidar's pointcloud|
|`lidar_frame_id`          |string   |frame ID for the lidar|
|`rviz_friendly`           |bool     |If True, swap the Y and Z positions and scale the reflectance of each data point for easier viewing in RViz. Causes increased latency|
|`lidar_sensor_pose`       |float[6] |pose is: position (X,Y,Z (meters)), orientation(pitch, roll, yaw (radians)). Pose transformation is CPU-expensive. Leave at 0.0 to disable|
|`use_system_default_qos`  |bool     |If False, publish data with sensor data QoS. Otherwise use default QoS determined by the RMW layer (useful for `rosbag2` recording).|
|`h3_proc_mask`            |string   |Mask encoding data processors to activate, default `PCL` |
|`buffer_size`    |int     |Size of the buffer allocated to store Lidar data. Default 12MB |


INSTALLATION
Extract the source repo to a ROS2 build directory.
The CMakeLists.txt file expects the LuminLib SDK to be pre-built into an .so library,
and located in an adjacent location, resulting in a directory structure of:

 [build-dir]
  |--LuminLib-all-2.16.0
  |--ros2_luminar
  
Edit the file ros2_luminar/CMakeLists.txt if your LuminLib is installed in a different location.


BUILD
Build the driver as you would any other ros2 application, with consideration for DEBUG or RELEASE optimization:  
Debug:  

    source your ros2 installation (such as 'source /opt/ros/foxy/setup.bash')
    colcon build --symlink-install

Release / Optimized:  

    source your ros2 installation (such as 'source /opt/ros/foxy/setup.bash')
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

Note that 2 copies of the file `libLuminLib.a` have been included under `./LuminLib-all-2.16.0/build` -- the smaller file is the optimized (Release) version, the other is a Debug version.  

SETUP  
Add the built driver to the ROS2 environment, as in:

    source install/setup.bash

RUNNING THE DRIVER
This driver uses ROS2 launch files and YAML configuration files.
To connect to multiple H3 Lidar units, a configuration .yaml file needs to be created for 
each unit, with the 'lidar_fingerprint' set to the ID number of each H3 Lidar sensor.

There are example files in:
    ros2_luminar/launch -- for the launch.py files
    ros2_luminar/params -- for the config.yaml files

Launch via command line, such as:
    ros2 launch ./install/ros2_luminar/share/ros2_luminar/launch/h3_f_launch.py

The example launch files will name the nodes and reference the config.yaml files
for each driver instance.   There are 3 example files, for FRONT, LEFT-REAR, and
RIGHT-REAR lidar sensors.  These files have the fingerprint values of the three
test lidar units that were driven on the track for data collection.

The example config.yaml files hold the startup settings for the driver and LiDAR sensor,
as ROS2 parameters.  Many of these parameters can be changed at runtime using 
'ros2 param' commands.

