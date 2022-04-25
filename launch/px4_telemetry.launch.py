import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
   config = os.path.join(
      get_package_share_directory('mavbase2'),
      'config',
      'px4_telemetry.yaml'
      )

   return LaunchDescription([
         Node(
            package='mavros',
            executable='mavros_node',
            parameters=[config]
         )
      ])