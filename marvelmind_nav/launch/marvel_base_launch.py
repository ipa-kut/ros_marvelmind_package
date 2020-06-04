from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
import sys

def generate_launch_description():
    return LaunchDescription([
        LifecycleNode(package='marvelmind_nav', node_executable='marvelmind_nav',
                      node_name='lc_marvel2', output='screen'),
    ])

def main(argv=sys.argv[1:]):
    print("Running main")