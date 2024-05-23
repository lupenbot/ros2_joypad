// Wrapper

#include "ros2_joypad/ros2_joypad.hpp"

int main(int argc, char** argv)
{
    // Init ROS
    rclcpp::init(argc, argv);

    std::shared_ptr<ros2_joypad::Joypad> joypad_node = std::make_shared<ros2_joypad::Joypad>();

    // Spinning!
    if(joypad_node->openDevice()) {
        rclcpp::spin(joypad_node);
    }
    return 0;

}