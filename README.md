# gripper_controller

## Installation
1) Clone this package
```
cd /path/to/ros2_ws/src
git clone git@github.com:bk1021/gripper_controller.git
cd gripper_controller
```
2) Install dependencies
```
sudo apt update
sudo apt install libgpiod-dev libyaml-cpp-dev
```
3) build
```
cd ../..
colcon build --packages-select gripper_controller
```

## Usage
> Note: Check servo configuration in `config/gripper_params.yaml`, rebuild after edit
```
ros2 launch gripper_controller gripper.launch.py
```
To open gripper
```
ros2 service call /set_gripper std_srvs/srv/SetBool "{data: true}"
```
To close gripper
```
ros2 service call /set_gripper std_srvs/srv/SetBool "{data: false}"
```

## Servo angle tuning
Runs a standalone servo tuning program that reads config and lets you input angles interactively
```
ros2 run gripper_controller servo_control
```
