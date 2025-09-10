#include "gripper_controller/servo_control.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <csignal>
#include <yaml-cpp/yaml.h>

ServoController* global_servo = nullptr;

void signal_handler(int signum)
{
    std::cout << "\nStopping PWM and exiting...\n";
    if (global_servo) global_servo->stop_pwm();
    std::exit(0);
}

int main()
{
    // Register Ctrl+C handler
    std::signal(SIGINT, signal_handler);

    // Default parameters
    std::string chip = "/dev/gpiochip1";
    int line = 14;
    double period = 0.02;
    int min_us = 500;
    int max_us = 2500;

    // Try to read config file from share directory
    try {
        YAML::Node config = YAML::LoadFile("/usr/share/gripper_controller/config/servo_params.yaml");
        if (config["gripper"]) {
            auto g = config["gripper"];
            if (g["gpio_chip"]) chip = g["gpio_chip"].as<std::string>();
            if (g["gpio_line"]) line = g["gpio_line"].as<int>();
            if (g["period"]) period = g["period"].as<double>();
            if (g["min_us"]) min_us = g["min_us"].as<int>();
            if (g["max_us"]) max_us = g["max_us"].as<int>();
        }
    } catch (...) {
        std::cout << "Config file not found or failed to parse, using default values.\n";
    }

    // Initialize servo
    ServoController servo(chip, line, period, min_us, max_us);
    global_servo = &servo;
    servo.start_pwm();

    std::cout << "Enter servo angle [0-180] repeatedly. Ctrl+C to exit.\n";

    while (true) {
        std::cout << "Angle> ";
        std::string line_input;
        if (!std::getline(std::cin, line_input)) break;

        std::istringstream iss(line_input);
        int angle;
        if (!(iss >> angle) || angle < 0 || angle > 180) {
            std::cout << "Invalid input. Enter integer 0-180.\n";
            continue;
        }

        servo.set_target_angle(angle);
        std::cout << "Setting angle to " << angle << "°...\n";
    }

    servo.stop_pwm();
    return 0;
}
