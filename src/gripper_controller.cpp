#include "gripper_controller/gripper_controller.hpp"
#include <chrono>
#include <thread>
#include <algorithm>

using namespace std::chrono_literals;

GripperController::GripperController() : Node("gripper_controller")
{
    // Declare parameters with defaults
    this->declare_parameter("open_angle", 38);
    this->declare_parameter("close_angle", 3);
    this->declare_parameter("gpio_chip", "/dev/gpiochip1");
    this->declare_parameter("gpio_line", 14);
    this->declare_parameter("period", 0.02);
    this->declare_parameter("min_us", 500);
    this->declare_parameter("max_us", 2500);

    open_angle_ = this->get_parameter("open_angle").as_int();
    close_angle_ = this->get_parameter("close_angle").as_int();
    gpio_chip_ = this->get_parameter("gpio_chip").as_string();
    gpio_line_ = this->get_parameter("gpio_line").as_int();
    period_ = this->get_parameter("period").as_double();
    min_us_ = this->get_parameter("min_us").as_int();
    max_us_ = this->get_parameter("max_us").as_int();

    // Init GPIO
    chip_ = gpiod_chip_open(gpio_chip_.c_str());
    if (!chip_) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open gpio chip: %s", gpio_chip_.c_str());
      rclcpp::shutdown();
      return;
    }
    line_ = gpiod_chip_get_line(chip_, gpio_line_);
    if (!line_) {
      RCLCPP_FATAL(this->get_logger(), "Failed to get gpio line %d from %s", gpio_line_, gpio_chip_.c_str());
      rclcpp::shutdown();
      return;
    }
    if (gpiod_line_request_output(line_, "gripper", 0) < 0) {
      RCLCPP_FATAL(this->get_logger(), "Failed to request line as output");
      rclcpp::shutdown();
      return;
    }

    // Create service
    service_ = this->create_service<std_srvs::srv::SetBool>(
      "set_gripper",
      std::bind(&GripperController::handle_request, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "GripperController ready. Service: /set_gripper");
}

GripperController::~GripperController()
{
    // cleanup GPIO
    gpiod_line_set_value(line_, 0);
    gpiod_line_release(line_);
    gpiod_chip_close(chip_);
}

double GripperController::angle_to_high_s(int angle)
{
    double us = min_us_ + (max_us_ - min_us_) * std::clamp(angle, 0, 180) / 180.0;
    return us / 1e6;
}

void GripperController::send_pulse(int angle)
{
    double high_s = angle_to_high_s(angle);
    for (int i = 0; i < 100; i++)
    {
        gpiod_line_set_value(line_, 1);
        std::this_thread::sleep_for(std::chrono::duration<double>(high_s));
        gpiod_line_set_value(line_, 0);
        std::this_thread::sleep_for(std::chrono::duration<double>(period_ - high_s));
    }
}

void GripperController::handle_request(
      const std_srvs::srv::SetBool::Request::SharedPtr request,
      std_srvs::srv::SetBool::Response::SharedPtr response)
{
    int target_angle = request->data ? open_angle_ : close_angle_;
    RCLCPP_INFO(this->get_logger(), "Moving gripper to %s (%d°)",
                request->data ? "OPEN" : "CLOSE", target_angle);

    send_pulse(target_angle);

    response->success = true;
    response->message = request->data ? "Gripper opened" : "Gripper closed";
}


