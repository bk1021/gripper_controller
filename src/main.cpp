#include "gripper_controller/gripper_controller.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<GripperController>();

    // Check if we're in tuning mode
    bool tuning_mode = node->get_parameter("tune").as_bool();

    if (tuning_mode) {
        // Run tuning mode (interactive console)
        node->run_tuning_mode();
    } else {
        // Run normal ROS node
        rclcpp::spin(node);
    }

    rclcpp::shutdown();
    return 0;
}