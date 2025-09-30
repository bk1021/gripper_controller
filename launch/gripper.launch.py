import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    bgthread_arg = DeclareLaunchArgument(
        'bgthread',
        default_value='false',
        description='Use background thread for PWM control'
    )
    
    tune_arg = DeclareLaunchArgument(
        'tune',
        default_value='false',
        description='Start in tuning mode for servo angle adjustment'
    )

    # Get package directory for parameter file
    config_dir = os.path.join(get_package_share_directory('gripper_controller'), 'config')
    param_file = os.path.join(config_dir, 'gripper_params.yaml')

    return LaunchDescription([
        bgthread_arg,
        tune_arg,
        Node(
            package="gripper_controller",
            executable="gripper_controller",
            name="gripper_controller",
            parameters=[
                param_file,
                {
                    'bgthread': LaunchConfiguration('bgthread'),
                    'tune': LaunchConfiguration('tune')
                }
            ],
            output="screen"
        )
    ])
