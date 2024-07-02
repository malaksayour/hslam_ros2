import os
from launch.substitutions import PathJoinSubstitution, TextSubstitution

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare




def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('hslam_ros2'),
        'include',
        'hslam_params.yaml'
        )

    return LaunchDescription([
        Node(
            package='hslam_ros2',
            executable='hslam_live',
            name='hslam_live',
            namespace='/',
            parameters=[config]
            )
    ])

