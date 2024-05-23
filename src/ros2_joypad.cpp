#include "ros2_joypad/ros2_joypad.hpp"

namespace ros2_joypad {

    Joypad::Joypad() :
    Node("joypad_node")
    {
        // Declare needed parameter.
        this->declare_parameter("joypad_device", joypad_device_);

    };

    Joypad::~Joypad() {
        close(file_descriptor_);
    }


    void Joypad::initROS() {

        // Declare & Get ROS parameters.
        this->declareROSParams();
        this->getROSParams();

        // Initialize ROS callbacks.
        ros_timer_read_device_ = this->create_wall_timer(
            std::chrono::milliseconds(joypad_refresh_rate_ms_),
            std::bind(&Joypad::readJoypadDataCallback, this)
        );

        publisher_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(pub_callback_rate_ms_),
            std::bind(&Joypad::publishJoypadDataCallback, this)
        );

        // Declare publishers.
        pub_joypad_ = this->create_publisher<sensor_msgs::msg::Joy>("~/data", 10);
        pub_twist_ = this->create_publisher<geometry_msgs::msg::Twist>("~/cmd_vel", 10);

        // Parse joypad mapping.
        this->parseMapping(joypad_buttons_mapping_ , joypad_map_, BUTTON);
        this->parseMapping(joypad_axis_mapping_    , joypad_map_, AXIS);
        this->parseMapping(joypad_dpad_mapping_    , joypad_map_, AXIS);
        
    }


    void Joypad::declareROSParams() {

        this->declare_parameter("buttons_mapping", std::vector<std::string>{"{\"BUTTON\", 0}"});
        this->declare_parameter("axis_mapping", std::vector<std::string>{"{\"AXIS\", 0}"});
        this->declare_parameter("dpad_mapping", std::vector<std::string>{"{\"DPAD\", 0}"});
        this->declare_parameter("left_analog_scale_x_axis", 1.0);
        this->declare_parameter("left_analog_scale_y_axis", 1.0);
        this->declare_parameter("right_analog_scale_x_axis", 1.0);
        this->declare_parameter("right_analog_scale_y_axis", 1.0);
        this->declare_parameter("z_left_scale_axis", 1.0);
        this->declare_parameter("z_right_scale_axis", 1.0);
        this->declare_parameter("frame_id", frame_id_);
        this->declare_parameter("joypad_refresh_rate_ms", 10);
        this->declare_parameter("pub_callback_rate_ms", 10);

        this->declare_parameter("publish_cmd_vel", publish_cmd_vel_);
        this->declare_parameter("twist_linear_axis_pos_tag", twist_linear_axis_pos_tag_);
        this->declare_parameter("twist_linear_axis_neg_tag", twist_linear_axis_neg_tag_);
        this->declare_parameter("twist_angular_axis_pos_tag", twist_angular_axis_pos_tag_);
        this->declare_parameter("twist_angular_axis_neg_tag", twist_angular_axis_neg_tag_);
        this->declare_parameter("twist_max_linear_axis_pos", twist_max_linear_axis_pos_);
        this->declare_parameter("twist_max_linear_axis_neg", twist_max_linear_axis_neg_);
        this->declare_parameter("twist_max_angular_axis_pos", twist_max_angular_axis_pos_);
        this->declare_parameter("twist_max_angular_axis_neg", twist_max_angular_axis_neg_);

    }


    void Joypad::getROSParams() {

        joypad_buttons_mapping_ =       this->get_parameter("buttons_mapping").as_string_array();
        joypad_axis_mapping_ =          this->get_parameter("axis_mapping").as_string_array();
        joypad_dpad_mapping_ =          this->get_parameter("dpad_mapping").as_string_array();   
        left_analog_scale_x_axis_ =     this->get_parameter("left_analog_scale_x_axis").as_double();
        left_analog_scale_y_axis_ =     this->get_parameter("left_analog_scale_y_axis").as_double();
        right_analog_scale_x_axis_ =    this->get_parameter("right_analog_scale_x_axis").as_double();
        right_analog_scale_y_axis_ =    this->get_parameter("right_analog_scale_y_axis").as_double();
        z_left_scale_axis_ =            this->get_parameter("z_left_scale_axis").as_double();
        z_right_scale_axis_ =           this->get_parameter("z_right_scale_axis").as_double();
        frame_id_ =                     this->get_parameter("frame_id").as_string();
        joypad_refresh_rate_ms_ =       this->get_parameter("joypad_refresh_rate_ms").as_int();
        pub_callback_rate_ms_ =         this->get_parameter("pub_callback_rate_ms").as_int();
    
        publish_cmd_vel_ =              this->get_parameter("publish_cmd_vel").as_bool();
        twist_linear_axis_pos_tag_ =    this->get_parameter("twist_linear_axis_pos_tag").as_string();
        twist_linear_axis_neg_tag_ =    this->get_parameter("twist_linear_axis_neg_tag").as_string();
        twist_angular_axis_pos_tag_ =   this->get_parameter("twist_angular_axis_pos_tag").as_string();
        twist_angular_axis_neg_tag_ =   this->get_parameter("twist_angular_axis_neg_tag").as_string();
        twist_max_linear_axis_pos_ =    this->get_parameter("twist_max_linear_axis_pos").as_double();
        twist_max_linear_axis_neg_ =    this->get_parameter("twist_max_linear_axis_neg").as_double();
        twist_max_angular_axis_pos_ =   this->get_parameter("twist_max_angular_axis_pos").as_double();
        twist_max_angular_axis_neg_ =   this->get_parameter("twist_max_angular_axis_neg").as_double();
    }


    bool Joypad::openDevice() {

        // Check 
        joypad_device_ = this->get_parameter("joypad_device").as_string();

        // Try to open
        file_descriptor_ = open(joypad_device_.c_str(), O_NONBLOCK);

        // Could not open joypad device.
        if(file_descriptor_ < 0) {

            RCLCPP_ERROR(this->get_logger(), " > Could not open %s device!", joypad_device_.c_str());
            RCLCPP_ERROR(this->get_logger(), " > Please check if joypad is connected or 'joypad_device' parameter is set correctly!");
            return false;

        } else {
            
            // Try to get joypad name.
            char device_name[128];
            if(ioctl(file_descriptor_, JSIOCGNAME(sizeof(device_name)), device_name) < 0) {
                std::strncpy(device_name, "Unknown", sizeof(device_name));
            }
            joypad_map_.joypad_name = std::string(device_name);
            RCLCPP_INFO(this->get_logger(), " > Device '%s' connected!", joypad_map_.joypad_name.c_str());

            // Set controller name as a ROS parameter.
            this->declare_parameter("controller_name", std::string(device_name));

            // Initialize ROS environment.
            init_ok_ = true;
            this->initROS();

            return true;
            
        }

    }


    void Joypad::readJoypadDataCallback() {

        while(read(file_descriptor_, &joypad_event_, sizeof(joypad_event_)) > 0) {

            switch(joypad_event_.type) {
                case JS_EVENT_INIT:
                    std::cout << "Init joypad\n";
                    break;
                    
                case JS_EVENT_BUTTON:

                    joypad_map_.button_mapping.at(joypad_event_.number).value = joypad_event_.value; 
                    // RCLCPP_INFO(this->get_logger(),
                    //     "Button %s pressed! Value: %d",
                    //     joypad_map_.button_mapping.at(joypad_event_.number).key.c_str(),
                    //     joypad_map_.button_mapping.at(joypad_event_.number).value);
                    break;
                    
                case JS_EVENT_AXIS:

                    joypad_map_.axis_mapping.at(joypad_event_.number).value = joypad_event_.value;
                    // RCLCPP_INFO(this->get_logger(),
                    //     "Button %s pressed! Value: %f",
                    //     joypad_map_.axis_mapping.at(joypad_event_.number).key.c_str(),
                    //     joypad_map_.axis_mapping.at(joypad_event_.number).value);

                    // Set twist message.
                    // TODO: To implement using switch case and not using strings.
                    // * Use only one tag for axis.
                    // * Add axis inversion.

                    // Linear
                    aux_range_ = twist_max_linear_axis_neg_ - twist_max_linear_axis_pos_;
                    if(joypad_map_.axis_mapping.at(joypad_event_.number).key == twist_linear_axis_pos_tag_){
                        ros_msg_twist_.linear.x = joypad_map_.axis_mapping.at(joypad_event_.number).value * 
                                                  (aux_range_) / 65535.0;
                    }
                    if(joypad_map_.axis_mapping.at(joypad_event_.number).key == twist_linear_axis_neg_tag_){
                        ros_msg_twist_.linear.x = joypad_map_.axis_mapping.at(joypad_event_.number).value * 
                                                  (aux_range_) / 65535.0;
                    }

                    // Angular
                    aux_range_ = twist_max_angular_axis_neg_ - twist_max_angular_axis_pos_;
                    if(joypad_map_.axis_mapping.at(joypad_event_.number).key == twist_angular_axis_pos_tag_){
                        ros_msg_twist_.angular.z = joypad_map_.axis_mapping.at(joypad_event_.number).value * 
                                                  (aux_range_) / 65535.0;
                    }
                    if(joypad_map_.axis_mapping.at(joypad_event_.number).key == twist_angular_axis_neg_tag_){
                        ros_msg_twist_.angular.z = joypad_map_.axis_mapping.at(joypad_event_.number).value * 
                                                  (aux_range_) / 65535.0;
                    }

                    break;

                default:
                    break;
            }
        }
        if(errno != EAGAIN) {
            RCLCPP_ERROR(this->get_logger(), " > Error while reading joypad data! Error No: %d", errno);
            rclcpp::shutdown();
        }

    }


    void Joypad::publishJoypadDataCallback() {

        // Data.
        InputMap::iterator it_begin;
        InputMap::iterator it_end;
        std::vector<float> axes;
        std::vector<int> buttons;
        
        // Push axes.
        it_begin = joypad_map_.axis_mapping.begin();
        it_end = joypad_map_.axis_mapping.end();
        for(; it_begin != it_end; it_begin++){
            axes.push_back(it_begin->second.value);
        }

        // Push buttons.
        it_begin = joypad_map_.button_mapping.begin();
        it_end = joypad_map_.button_mapping.end();
        for(; it_begin != it_end; it_begin++){
            buttons.push_back(it_begin->second.value);
        }
        
        // Build ROS msg.
        // Headers.
        ros_msg_joypad_.header.frame_id = frame_id_;
        ros_msg_joypad_.header.stamp = this->now();
        
        ros_msg_joypad_.axes = axes;
        ros_msg_joypad_.buttons = buttons;

        // Publish data.
        pub_joypad_->publish(ros_msg_joypad_);
        if(publish_cmd_vel_) pub_twist_->publish(ros_msg_twist_);

    }


    void Joypad::parseMapping(std::vector<std::string> & string_tuple, JoypadDescription &joypad_description, uint8_t type) {

        std::vector<std::string>::iterator it_begin;
        std::vector<std::string>::iterator it_end;

        std::string current_map = "";

        // Parse buttons.
        it_begin = string_tuple.begin();
        it_end = string_tuple.end();
        for(; it_begin!=it_end; it_begin++) {
            current_map = *it_begin;

            // Find delimiter characters.
            std::size_t left_brace_index = current_map.find('{');
            std::size_t right_brace_index = current_map.find('}');
            std::size_t comma_index = current_map.find(',');
            if( std::string::npos == right_brace_index || 
                std::string::npos == left_brace_index ||
                std::string::npos == comma_index) {
                RCLCPP_ERROR(this->get_logger(), " > Key '%s' broken!", current_map.c_str());    
                continue;
            }

            // Get key-value pair.
            std::string key = current_map.substr(left_brace_index + 1, comma_index - 1);
            uint8_t value = 254;
            try {
                value = (uint8_t)std::stoi(current_map.substr(comma_index + 1, right_brace_index));
            } catch(std::exception e) {
                RCLCPP_ERROR(this->get_logger(), " > Key '%s' broken!", current_map.c_str());
                continue;
            }

            // Store pair un joypad map.

            ButtonStatus default_status;
            default_status.key = key; // String key.

            InputPair button_pair;
            button_pair.second = default_status;
            button_pair.first = value; // Button number key.

            switch (type) {
                case BUTTON:
                    joypad_description.button_mapping.insert(button_pair);
                    // RCLCPP_INFO(this->get_logger(), joypad_description.button_mapping.at(value).key.c_str());
                    break;
                
                case AXIS:
                    joypad_description.axis_mapping.insert(button_pair);
                    // RCLCPP_INFO(this->get_logger(), joypad_description.axis_mapping.at(value).key.c_str());
                    break;
                
                default:
                    break;
            }
            
            RCLCPP_INFO(this->get_logger(), " > Mapping registered: %s", current_map.c_str());

        }

    }

};