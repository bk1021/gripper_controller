import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="gripper_controller",
            executable="gripper_controller",
            name="gripper_controller",
            parameters=[os.path.join(get_package_share_directory('gripper_controller'), 'config', 'gripper_params.yaml')],
            output="screen"
        )
    ])
