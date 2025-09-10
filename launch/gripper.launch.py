from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="gripper_controller",
            executable="gripper_controller",
            name="gripper_controller",
            parameters=["config/gripper_params.yaml"],
            output="screen"
        )
    ])
