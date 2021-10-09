# aptiv_esr_ros2
ROS2 driver for Aptiv ESR RADAR, using ros2_socketcan

To Build:  
Clone the following into a working directory:  
 - ros_canopen (dashing-devel branch)  
   - git clone --single-branch --branch dashing-devel https://githubcom/ros-industrial/ros_canopen.git
 - ros2_socketcan  
   - git clone https://github.com/autowarefoundation/ros2_socketcan.git
 - astuff_sensor_msgs (dashing-devel branch)
   - git clone --single-branch --branch dashing-devel https://github.com/astuff/astuff_sensor_msgs

Clone this repository into the same working directory
 - git clone https://github.com/neil-rti/aptiv_esr_ros2.git

Source ros2, such as:
 - source /opt/ros/foxy/setup.bash

Build all:  
 - colcon build --symlink-install


To Run:  
The CAN bus must be accessible as a socket connection.  
To configure this:

    sudo modprobe can  
    sudo modprobe can_raw  
    sudo ip link set can0 type can bitrate 500000  
    sudo ip link set up can0

Launch the ros2 socketcan node:
 - source install/setup.bash
 - ros2 launch ./ros2_socketcan/launch/socket_can_bridge.launch.xml

Launch the ESR driver
 - source install/setup.bash
 - ros2 run aptiv_esr_ros2 esr_driver


This will communicate with the ros2 can bus driver using data type `can_msgs::msg::Frame`
on topics `from_can_bus` and `to_can_bus`.  
These CAN bus frame topics are further divided into separate topics for control and status:  

    delphi_esr_msgs::msg::EsrVehicle1 to EsrVehicle5
    delphi_esr_msgs::msg::EsrStatus1 to EsrStatus9
    delphi_esr_msgs::msg::EsrValid1 to EsrValid2
    delphi_esr_msgs::msg::EsrTrack

