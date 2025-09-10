#include "gripper_controller/servo_control.hpp"
#include <chrono>
#include <thread>
#include <algorithm>
#include <iostream>

ServoController::ServoController(const std::string& chip_path, int line_num,
                                 double period, int min_us, int max_us)
    : period_(period), min_us_(min_us), max_us_(max_us), target_angle_(90), running_(false)
{
    chip_ = gpiod_chip_open(chip_path.c_str());
    if (!chip_) throw std::runtime_error("Failed to open GPIO chip");

    line_ = gpiod_chip_get_line(chip_, line_num);
    if (!line_) throw std::runtime_error("Failed to get GPIO line");

    if (gpiod_line_request_output(line_, "servo", 0) < 0)
        throw std::runtime_error("Failed to request line as output");
}

ServoController::~ServoController()
{
    stop_pwm();
    gpiod_line_set_value(line_, 0);
    gpiod_line_release(line_);
    gpiod_chip_close(chip_);
}

void ServoController::set_target_angle(int angle)
{
    target_angle_ = std::clamp(angle, 0, 180);
}

double ServoController::angle_to_high_s(int angle)
{
    angle = std::clamp(angle, 0, 180);
    double us = min_us_ + (max_us_ - min_us_) * angle / 180.0;
    return us / 1e6;
}

void ServoController::pulse(double high_s)
{
    gpiod_line_set_value(line_, 1);
    std::this_thread::sleep_for(std::chrono::duration<double>(high_s));
    gpiod_line_set_value(line_, 0);
    std::this_thread::sleep_for(std::chrono::duration<double>(period_ - high_s));
}

void ServoController::pwm_loop()
{
    while (running_) {
        double high_s = angle_to_high_s(target_angle_);
        pulse(high_s);
    }
}

void ServoController::start_pwm()
{
    if (!running_) {
        running_ = true;
        pwm_thread_ = std::thread(&ServoController::pwm_loop, this);
    }
}

void ServoController::stop_pwm()
{
    if (running_) {
        running_ = false;
        if (pwm_thread_.joinable()) pwm_thread_.join();
    }
}
