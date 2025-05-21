from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    turtlebot4_nav_share = FindPackageShare('turtlebot4_navigation').find('turtlebot4_navigation')

    localization_launch = os.path.join(turtlebot4_nav_share, 'launch', 'localization.launch.py')
    nav2_launch = os.path.join(turtlebot4_nav_share, 'launch', 'nav2.launch.py')

    map_file = LaunchConfiguration('map', default='office.yaml')

    return LaunchDescription([
        # Launch localization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={'map': map_file}.items(),
        ),

        # Launch nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
        ),
    ])
