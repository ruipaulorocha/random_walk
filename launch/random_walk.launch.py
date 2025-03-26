#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=TextSubstitution(text = 'False'),
        description='Whether to use simulated time')

    # launch configuration variables
    this_pkg_directory = get_package_share_directory('random_walk')
    param_file = os.path.join(this_pkg_directory, 'config', 'random_walk.yaml')

    # specify the actions
    set_use_sim_time = launch_ros.actions.SetParameter(name = 'use_sim_time',
        value = LaunchConfiguration('use_sim_time'))
    launch_brownian_motion = Node(
        package =       'random_walk',
        executable =    'random_walk',
        name =          'random_walk',
        namespace =     '',
        parameters = [param_file]
    )

    # create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(set_use_sim_time)
    ld.add_action(launch_brownian_motion)

    return ld
