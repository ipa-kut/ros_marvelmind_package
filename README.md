# Marvelmind ROS2 Package

This package is a port of the [ros_marvelmind_package](https://bitbucket.org/marvelmind_robotics/ros_marvelmind_package/src/master/)
from ROS to ROS2. All rights, as mentioned by the License, reserved by Marvelmind Robotics.

# Installation from source
Simply clone this package to your workspace inside the src directory..    
Then from the root of your workspace, install all dependencies with 
```
rosdep install --from-paths src --ignore-src --rosdistro eloquent -r -y
```
And then build with
```
colcon build --symlink-install
```
This package was developed on and tested on ROS2 Eloquent running on Ubuntu 18.04.   

## Prerequisites
1. Ensure user is added to `dialout` group to get access to serial port. (needs a reboot)    
2. Follow the instructions on [this page](https://marvelmind.com/pics/marvelmind_ROS.pdf) as well to ensure the same access.
3. You can also add a file called `99-tty.rules` under `/etc/udev/rules.d` with the following content:
```
#Marvelmind serial port rules
KERNEL==”ttyACM0”,GROUP=”dialout”,MODE=”666”
```

## Bringup

Once you have ensured that a beacon or modem is connected to your PC, launch with:

`ros2 launch marvelmind_nav marvel_driver_launch.py`   
To change the port, edit the launch file and replace it in [this line](https://github.com/ipa-kut/ros_marvelmind_package/blob/a032ac60ac72a85ef5d4dfa5bee3d10e265fd9d8/marvelmind_nav/launch/marvel_driver_launch.py#L28) in the launch file.

