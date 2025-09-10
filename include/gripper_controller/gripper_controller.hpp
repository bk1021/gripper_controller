#pragma once

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <gpiod.h>

class GripperController : public rclcpp::Node
{
public:
    GripperController();
    ~GripperController();

private:
    // Member variables
    int open_angle_, close_angle_, gpio_line_, min_us_, max_us_;
    double period_;
    std::string gpio_chip_;
    gpiod_chip *chip_;
    gpiod_line *line_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;

    // Function declarations
    double angle_to_high_s(int angle);
    void send_pulse(int angle);
    void handle_request(
        const std_srvs::srv::SetBool::Request::SharedPtr request,
        const std_srvs::srv::SetBool::Response::SharedPtr response);
};
