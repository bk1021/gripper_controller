# gripper_controller

## Installation
1) Clone this package
```
cd /path/to/ros2_ws/src
git clone https://github.com/bk1021/gripper_controller.git
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

Run `gripper_controller` node in default mode (PWM signal is send for 100 * `period` s after service call/angle command published) 
```
ros2 launch gripper_controller gripper.launch.py
```
For using continuous PWM signal control
```
ros2 launch gripper_controller gripper.launch.py bgthread:=true
```
To open gripper
```
ros2 service call /set_gripper std_srvs/srv/SetBool "{data: true}"
```
To close gripper
```
ros2 service call /set_gripper std_srvs/srv/SetBool "{data: false}"
```
Direct angle control (0 - 180 deg) (not recommended)
```
ros2 topic pub /gripper_angle std_msgs/msg/Int32 "{data: 90}"
```

## Servo angle tuning
Run a interactive console for servo motor angle tuning 
```
ros2 launch gripper_controller gripper.launch.py tune:=true
```
