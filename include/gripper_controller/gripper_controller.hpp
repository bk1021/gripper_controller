#pragma once

#include <string>
#include <thread>
#include <atomic>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include <gpiod.h>

class GripperController : public rclcpp::Node
{
public:
    GripperController();
    ~GripperController();

    void run_tuning_mode();

private:
    // ROS argument/parameters
    bool use_bgthread_;
    bool tuning_mode_;
    int open_angle_, close_angle_, gpio_line_, min_us_, max_us_;
    double period_;
    std::string gpio_chip_;
    
    // GPIO hardware
    gpiod_chip *chip_;
    gpiod_line *line_;
    
    // ROS interfaces
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr angle_sub_;
    
    // Background thread for PWM
    std::thread pwm_thread_;
    std::atomic<bool> running_;
    std::atomic<int> target_angle_;

    // Methods
    void initialize_gpio();
    void initialize_ros_interfaces();
    
    // PWM control methods
    double angle_to_high_s(int angle);
    void send_pulse(int angle);
    void pwm_loop();
    
    // ROS callback methods
    void handle_gripper_request(
        const std_srvs::srv::SetBool::Request::SharedPtr request,
        const std_srvs::srv::SetBool::Response::SharedPtr response);
    
    void handle_angle_command(const std_msgs::msg::Int32::SharedPtr msg);
};