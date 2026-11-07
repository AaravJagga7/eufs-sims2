#!/usr/bin/env python3
"""Launch script for ROS 2 Driverless Simulation."""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.abspath(__file__))

    rate_arg = DeclareLaunchArgument(
        "rate_hz",
        default_value="20.0",
        description="Simulation and perception publish frequency in Hz"
    )

    autonomous_arg = DeclareLaunchArgument(
        "autonomous_mode",
        default_value="True",
        description="Whether the joy controller drives autonomously"
    )

    duration_arg = DeclareLaunchArgument(
        "max_duration_sec",
        default_value="0.0",
        description="Max runtime in seconds (0.0 = unlimited)"
    )

    sim_node = Node(
        package="driverless_sim",
        executable="ros_driverless_sim.py",
        name="driverless_sim_node",
        output="screen",
        parameters=[{
            "rate_hz": LaunchConfiguration("rate_hz"),
            "autonomous_mode": LaunchConfiguration("autonomous_mode"),
            "max_duration_sec": LaunchConfiguration("max_duration_sec")
        }]
    )

    return LaunchDescription([
        rate_arg,
        autonomous_arg,
        duration_arg,
        sim_node
    ])

