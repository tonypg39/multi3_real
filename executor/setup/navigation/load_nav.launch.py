from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    namespace = LaunchConfiguration('namespace', default='/Turtlebot_02490')
    map_file = LaunchConfiguration('map', default='/home/ubuntu/navigation/lab_corridor.yaml')

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot4_navigation'),
                'launch',
                'localization.launch.py'
            ])
        ]),
        launch_arguments={
            'namespace': namespace,
            'map': map_file,
        }.items()
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot4_navigation'),
                'launch',
                'nav2.launch.py'
            ])
        ]),
        launch_arguments={
            'namespace': namespace,
        }.items()
    )

    return LaunchDescription([
        localization_launch,
        nav2_launch,
    ])
