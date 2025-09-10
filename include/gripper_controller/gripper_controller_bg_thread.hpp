#pragma once

#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <gpiod.h>

class GripperController : public rclcpp::Node
{
public:
    GripperController();
    ~GripperController();

private:
    // ROS service
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;

    // GPIO
    gpiod_chip *chip_;
    gpiod_line *line_;
    int gpio_line_;
    std::string gpio_chip_;

    // PWM parameters
    double period_;
    int min_us_, max_us_;
    int open_angle_, close_angle_;

    // Background thread
    std::thread pwm_thread_;
    std::atomic<bool> running_;
    std::atomic<int> target_angle_;

    // Methods
    void handle_request(
        const std_srvs::srv::SetBool::Request::SharedPtr request,
        const std_srvs::srv::SetBool::Response::SharedPtr response);

    double angle_to_high_s(int angle);
    void pwm_loop();
};
