#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # General variables.
    namespace = 'joypads'
    this_package_name = 'ros2_joypad'
    this_node_name='SN30ProPlus'

    # YAML configuration paths.
    node_params_path = os.path.join(
        get_package_share_directory(this_package_name),
        'params',
        '8bitdo_SN30ProPlus.yaml'
    )
    
    # Nodes
    joypad_node = Node(
        package=this_package_name,
        namespace=namespace,
        executable='joypad',
        name=this_node_name,
        parameters=[
            node_params_path
        ],
        # remappings=[
        #     ('~/cmd_vel', '/to/new/cmd_vel/topic'),
        #     ('~/data', '/to/new/data/topic'),        
        # ],
        output='screen'
    )
    
    # Launcher
    return LaunchDescription([
        joypad_node
    ])