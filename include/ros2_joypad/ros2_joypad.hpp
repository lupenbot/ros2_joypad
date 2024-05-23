/**
 * @file ros2_joypad.hpp
 * @author Lupenbot (https://github.com/lupenbot/ros2_joypad)
 * @brief 
 * 
 * References: 
 * - https://www.kernel.org/doc/Documentation/input/joystick-api.txt
 * - https://github.com/torvalds/linux/blob/master/include/uapi/linux/joystick.h
 * @version 1.0
 * @date 2024-05-22
 * 
 * @copyright MIT (c) 2024
 * 
 */


// General libs
#include <iostream>
#include <string>
#include <cstring>
#include <chrono>
#include <functional>
#include <memory>
#include <vector>

// Joypad
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include "linux/joystick.h"

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace ros2_joypad {

    enum JOYPAD_INPUT_TYPE {
        BUTTON = 0,
        AXIS = 1
    };

    struct ButtonStatus {
        std::string key = "input";
        float value = 0.0;
    };
    typedef std::pair<uint8_t, ButtonStatus> InputPair;
    typedef std::map<uint8_t, ButtonStatus> InputMap;

    struct JoypadDescription {
        std::string joypad_name = "";
        InputMap button_mapping;
        InputMap axis_mapping;
    };


    class Joypad : public rclcpp::Node {

        public:
            Joypad();
            ~Joypad();

            bool openDevice();

        private:


            // Joypad variables.
            bool init_ok_ = false;
            std::unique_ptr<const char> p_joypad_input_device_;
            std::string joypad_device_ = "/dev/input/jsX";
            int64_t joypad_refresh_rate_ms_ = 10; 
            int  file_descriptor_ = 0;

            struct js_event joypad_event_;
            std::vector<std::string> joypad_buttons_mapping_;
            std::vector<std::string> joypad_axis_mapping_;
            std::vector<std::string> joypad_dpad_mapping_;

            double left_analog_scale_x_axis_ = 1.0;
            double left_analog_scale_y_axis_ = 1.0;
            double right_analog_scale_x_axis_ = 1.0;
            double right_analog_scale_y_axis_ = 1.0;
            double z_left_scale_axis_ = 1.0;
            double z_right_scale_axis_ = 1.0;
            
            bool publish_cmd_vel_ = false;
            std::string twist_linear_axis_pos_tag_ = "";
            std::string twist_linear_axis_neg_tag_ = "";
            std::string twist_angular_axis_pos_tag_ = "";
            std::string twist_angular_axis_neg_tag_ = "";

            double aux_range_ = 0;
            double twist_max_linear_axis_pos_ = 1.0;
            double twist_max_linear_axis_neg_ = -1.0;
            double twist_max_angular_axis_pos_ = 1.0;
            double twist_max_angular_axis_neg_ = -1.0;

            JoypadDescription joypad_map_;

            // ROS variables.
            std::string frame_id_ = "joypad";
            sensor_msgs::msg::Joy ros_msg_joypad_;
            geometry_msgs::msg::Twist ros_msg_twist_;

            int64_t pub_callback_rate_ms_ = 10;
            rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_joypad_;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist_;

            rclcpp::TimerBase::SharedPtr ros_timer_read_device_;
            void readJoypadDataCallback();

            rclcpp::TimerBase::SharedPtr publisher_timer_;
            void publishJoypadDataCallback();

            void declareROSParams();
            void getROSParams();

            // Aux.
            void initROS();
            void parseMapping(std::vector<std::string> & string_tuple, JoypadDescription &joypad_description, uint8_t type = BUTTON);
        
    };

};