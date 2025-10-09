#include "gripper_controller/gripper_controller.hpp"
#include <chrono>
#include <thread>
#include <algorithm>
#include <sstream>

using namespace std::chrono_literals;

GripperController::GripperController() 
    : Node("gripper_controller"), running_(false)
{
    // Declare ROS parameters
    this->declare_parameter("bgthread", false);
    this->declare_parameter("open_angle", 38);
    this->declare_parameter("close_angle", 3);
    this->declare_parameter("gpio_chip", "/dev/gpiochip1");
    this->declare_parameter("gpio_line", 14);
    this->declare_parameter("period", 0.02);
    this->declare_parameter("min_us", 500);
    this->declare_parameter("max_us", 2500);
    this->declare_parameter("min_angle", 0);
    this->declare_parameter("max_angle", 270);

    // Get parameters
    use_bgthread_ = this->get_parameter("bgthread").as_bool();
    open_angle_ = this->get_parameter("open_angle").as_int();
    close_angle_ = this->get_parameter("close_angle").as_int();
    gpio_chip_ = this->get_parameter("gpio_chip").as_string();
    gpio_line_ = this->get_parameter("gpio_line").as_int();
    period_ = this->get_parameter("period").as_double();
    min_us_ = this->get_parameter("min_us").as_int();
    max_us_ = this->get_parameter("max_us").as_int();
    min_angle_ = this->get_parameter("min_angle").as_int();
    max_angle_ = this->get_parameter("max_angle").as_int();

    // Initialize GPIO
    initialize_gpio();

    // Initialize ROS interfaces
    initialize_ros_interfaces();

    // Set initial target angle
    target_angle_ = open_angle_;

    // Start background thread if requested
    if (use_bgthread_) {
        running_ = true;
        pwm_thread_ = std::thread(&GripperController::pwm_loop, this);
        RCLCPP_INFO(this->get_logger(), "GripperController running in background thread mode");
    }

    RCLCPP_INFO(this->get_logger(), "GripperController ready. Mode: %s", 
                use_bgthread_ ? "background thread" : "normal");
}

GripperController::~GripperController()
{
    // Stop background thread if running
    if (running_) {
        running_ = false;
        if (pwm_thread_.joinable()) {
            pwm_thread_.join();
        }
    }

    // Cleanup GPIO
    if (line_) {
        gpiod_line_set_value(line_, 0);
        gpiod_line_release(line_);
    }
    if (chip_) {
        gpiod_chip_close(chip_);
    }
}

void GripperController::initialize_gpio()
{
    chip_ = gpiod_chip_open(gpio_chip_.c_str());
    if (!chip_) {
        RCLCPP_FATAL(this->get_logger(), "Failed to open gpio chip: %s", gpio_chip_.c_str());
        rclcpp::shutdown();
        return;
    }

    line_ = gpiod_chip_get_line(chip_, gpio_line_);
    if (!line_) {
        RCLCPP_FATAL(this->get_logger(), "Failed to get gpio line %d from %s", 
                     gpio_line_, gpio_chip_.c_str());
        rclcpp::shutdown();
        return;
    }

    if (gpiod_line_request_output(line_, "gripper", 0) < 0) {
        RCLCPP_FATAL(this->get_logger(), "Failed to request line as output");
        rclcpp::shutdown();
        return;
    }
}

void GripperController::initialize_ros_interfaces()
{
    // Create service for gripper control
    service_ = this->create_service<std_srvs::srv::SetBool>(
        "set_gripper",
        std::bind(&GripperController::handle_gripper_request, this,
            std::placeholders::_1, std::placeholders::_2));

    // Create subscription for angle commands
    angle_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "gripper_angle", 10,
        std::bind(&GripperController::handle_angle_command, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "ROS interfaces initialized. Service: /set_gripper, Topic: /gripper_angle");
}

double GripperController::angle_to_high_s(int angle)
{
    double us = std::clamp(
        min_us_ + (max_us_ - min_us_) * static_cast<double>(std::clamp(angle, min_angle, max_angle)) / (max_angle - min_angle), 
        static_cast<double>(min_us_),
        static_cast<double>(max_us_));
    return us / 1e6;
}

void GripperController::send_pulse(int angle)
{
    double high_s = angle_to_high_s(angle);
    for (int i = 0; i < 100; i++) {
        gpiod_line_set_value(line_, 1);
        std::this_thread::sleep_for(std::chrono::duration<double>(high_s));
        gpiod_line_set_value(line_, 0);
        std::this_thread::sleep_for(std::chrono::duration<double>(period_ - high_s));
    }
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

void GripperController::handle_gripper_request(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
{
    int new_angle = request->data ? open_angle_ : close_angle_;
    
    RCLCPP_INFO(this->get_logger(), "Moving gripper to %s (%d°)",
                request->data ? "OPEN" : "CLOSE", new_angle);

    if (use_bgthread_) {
        // In background thread mode, just update the target
        target_angle_ = new_angle;
    } else {
        // In normal mode, send pulse directly
        send_pulse(new_angle);
    }

    response->success = true;
    response->message = request->data ? "Gripper opened" : "Gripper closed";
}

void GripperController::handle_angle_command(const std_msgs::msg::Int32::SharedPtr msg)
{
    int angle = std::clamp(msg->data, min_angle_, max_angle_);
    
    RCLCPP_INFO(this->get_logger(), "Setting gripper angle to %d°", angle);

    if (use_bgthread_) {
        target_angle_ = angle;
    } else {
        send_pulse(angle);
    }
}
