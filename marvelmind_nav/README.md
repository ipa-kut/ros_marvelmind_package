# Marvelmind ROS2 Package

This package is an attempt to port the [ros_marvelmind_package](https://bitbucket.org/marvelmind_robotics/ros_marvelmind_package/src/master/)
from ROS to ROS2. All rights as mentioned by the License belong to Marvelmind.


## TODOS
1. Upgrade launch file to use LaunchCOnfiguration/LaunchArguments
2. Upgrade port and baud to params and load on `on_configure()` to make it more robust. Can be done once [this issue](https://github.com/ros2/rclcpp/issues/855) is fixed
