#include "gripper_controller/gripper_controller.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GripperController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}