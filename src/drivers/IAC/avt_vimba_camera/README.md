# avt_vimba_camera

This repo contains a ROS2 driver for cameras manufactured by [Allied Vision Technologies](https://www.alliedvision.com).  
The driver relies on libraries provided by AVT as part of their [Vimba SDK](https://www.alliedvision.com/en/products/software.html).  Ensure this SDK is available and the supporting transport libs have been installed in the machine that will run this ROS2 driver.  
This driver was ported from the ROS(1) driver at [AutonomouStuff GitHub](https://github.com/astuff/avt_vimba_camera) to ROS2, as of the Foxy release.

## Building the driver

Driver can be built by ensuring the [ROS2 image_common](https://github.com/ros-perception/image_common/tree/ros2) 
repo is installed and built, cloning this repository, then installing the Vimba SDK (default location is ./Vimba_4_2 in this repository; edit the CMakeLists.txt file to point to the SDK installation
location if installed in a different location).  
Then source your ROS2 installation and build as usual.  
For example:
```
git clone https://github.com/<this-repo>/avt_vimba_camera.git
cd avt_vimba_camera
(copy the Vimba SDK to ./Vimba_4_2, or edit CMakeLists.txt to point to alternate SDK location)
source (ros2 installation)
source (image_common)
colcon build --symlink-install
```

The driver can then be run as a ros2 package/executable, with support for pre-loaded configuration parameters.  
Such as:
```
ros2 run avt_vimba_camera mono_camera_node --ros-args --params-file ./config/mono_c1.yaml
```

