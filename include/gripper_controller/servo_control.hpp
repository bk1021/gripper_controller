#pragma once

#include <string>
#include <thread>
#include <atomic>
#include <gpiod.h>

class ServoController
{
public:
    ServoController(const std::string& chip = "/dev/gpiochip1",
                    int line = 14,
                    double period = 0.02,
                    int min_us = 500,
                    int max_us = 2500);

    ~ServoController();

    void set_target_angle(int angle);
    void start_pwm();
    void stop_pwm();

private:
    gpiod_chip* chip_;
    gpiod_line* line_;
    double period_;
    int min_us_;
    int max_us_;

    std::atomic<int> target_angle_;
    std::atomic<bool> running_;
    std::thread pwm_thread_;

    double angle_to_high_s(int angle);
    void pwm_loop();
    void pulse(double high_s);
};
