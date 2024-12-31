from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os
def generate_launch_description():

    return LaunchDescription([

        IncludeLaunchDescription(
         PythonLaunchDescriptionSource(
               os.path.join(
                  get_package_share_directory('autodrive_f1tenth'),
                     'launch', 'simulator_bringup_headless.launch.py')),
      ),
        Node(
            package='autodrive_mapping',
            executable='publish_odom',
            name='autodrive_bridge',
            emulate_tty=True,
            output='screen',
        )
    ])