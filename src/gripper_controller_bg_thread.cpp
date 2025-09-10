#include "gripper_controller/gripper_controller_bg_thread.hpp"
#include <chrono>
#include <thread>
#include <algorithm>

GripperController::GripperController() : Node("gripper_controller")
{
    // Parameters
    declare_parameter("open_angle", 38);
    declare_parameter("close_angle", 3);
    declare_parameter("gpio_chip", "/dev/gpiochip1");
    declare_parameter("gpio_line", 14);
    declare_parameter("period", 0.02);
    declare_parameter("min_us", 500);
    declare_parameter("max_us", 2500);

    open_angle_ = get_parameter("open_angle").as_int();
    close_angle_ = get_parameter("close_angle").as_int();
    gpio_chip_ = get_parameter("gpio_chip").as_string();
    gpio_line_ = get_parameter("gpio_line").as_int();
    period_ = get_parameter("period").as_double();
    min_us_ = get_parameter("min_us").as_int();
    max_us_ = get_parameter("max_us").as_int();

    // Initialize GPIO
    chip_ = gpiod_chip_open(gpio_chip_.c_str());
    if (!chip_) {
        RCLCPP_FATAL(get_logger(), "Failed to open gpio chip: %s", gpio_chip_.c_str());
        rclcpp::shutdown();
        return;
    }
    line_ = gpiod_chip_get_line(chip_, gpio_line_);
    if (!line_) {
        RCLCPP_FATAL(get_logger(), "Failed to get gpio line %d", gpio_line_);
        rclcpp::shutdown();
        return;
    }
    if (gpiod_line_request_output(line_, "gripper", 0) < 0) {
        RCLCPP_FATAL(get_logger(), "Failed to request line as output");
        rclcpp::shutdown();
        return;
    }

    // Initialize target
    target_angle_ = close_angle_;
    running_ = true;

    // Start PWM thread
    pwm_thread_ = std::thread(&GripperController::pwm_loop, this);

    // Create ROS service
    service_ = create_service<std_srvs::srv::SetBool>(
        "set_gripper",
        std::bind(&GripperController::handle_request, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "GripperController ready. Service: /set_gripper");
}

GripperController::~GripperController()
{
    // Stop thread
    running_ = false;
    if (pwm_thread_.joinable()) pwm_thread_.join();

    // Reset GPIO
    gpiod_line_set_value(line_, 0);
    gpiod_line_release(line_);
    gpiod_chip_close(chip_);
}

double GripperController::angle_to_high_s(int angle)
{
    angle = std::clamp(angle, 0, 180);
    double us = min_us_ + (max_us_ - min_us_) * angle / 180.0;
    return us / 1e6;
}

void GripperController::pwm_loop()
{
    while (running_) {
        double high_s = angle_to_high_s(target_angle_);
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
    target_angle_ = request->data ? open_angle_ : close_angle_;
    RCLCPP_INFO(get_logger(), "Gripper moving to %s (%d°)",
                request->data ? "OPEN" : "CLOSE", target_angle_.load());

    response->success = true;
    response->message = request->data ? "Gripper opened" : "Gripper closed";
}
