#!/usr/bin/env python3
"""
Autonomous Navigation Launch File
================================

Launches the full autonomous navigation system including:
- Robot model and sensors (LIDAR + OAK-D Lite)
- Autonomous navigation node
- Motor controller bridge

Usage:
  ros2 launch labrobot pi.autonomous.launch.py

Parameters:
  - micropython_ip: IP address of MicroPython controller (default: 192.168.25.72)
  - micropython_port: Port of MicroPython controller (default: 8080)
  - autonomous_enabled: Start with autonomous mode enabled (default: true)
  - min_obstacle_distance: Minimum distance to obstacles in meters (default: 0.5)
  - max_speed: Maximum motor speed percentage (default: 70)
  - scan_angle_range: LIDAR scan angle range in degrees (default: 90)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Package directory
    pkg_share = FindPackageShare('labrobot')
    
    # Launch arguments
    micropython_ip_arg = DeclareLaunchArgument(
        'micropython_ip',
        default_value='192.168.25.72',
        description='IP address of MicroPython motor controller'
    )
    
    micropython_port_arg = DeclareLaunchArgument(
        'micropython_port',
        default_value='8080',
        description='Port of MicroPython motor controller'
    )
    
    autonomous_enabled_arg = DeclareLaunchArgument(
        'autonomous_enabled',
        default_value='true',
        description='Start with autonomous navigation enabled'
    )
    
    min_obstacle_distance_arg = DeclareLaunchArgument(
        'min_obstacle_distance',
        default_value='0.5',
        description='Minimum distance to obstacles in meters'
    )
    
    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='70',
        description='Maximum motor speed percentage'
    )
    
    default_speed_arg = DeclareLaunchArgument(
        'default_speed',
        default_value='50',
        description='Default motor speed percentage'
    )
    
    rotation_speed_arg = DeclareLaunchArgument(
        'rotation_speed',
        default_value='40',
        description='Rotation speed percentage'
    )
    
    scan_angle_range_arg = DeclareLaunchArgument(
        'scan_angle_range',
        default_value='90',
        description='LIDAR scan angle range in degrees (±45° from front)'
    )
    
    # Include the full sensor launch (LIDAR + OAK-D Lite + IMU)
    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_share,
                'launch',
                'pi.full.launch.py'
            ])
        ]),
        launch_arguments={
            'use_rviz': 'false'  # Don't start RViz automatically in autonomous mode
        }.items()
    )
    
    # Autonomous navigation node
    autonomous_nav_node = Node(
        package='labrobot',
        executable='autonomous_navigation_node.py',
        name='autonomous_navigation_node',
        output='screen',
        parameters=[{
            'micropython_ip': LaunchConfiguration('micropython_ip'),
            'micropython_port': LaunchConfiguration('micropython_port'),
            'autonomous_enabled': LaunchConfiguration('autonomous_enabled'),
            'min_obstacle_distance': LaunchConfiguration('min_obstacle_distance'),
            'max_speed': LaunchConfiguration('max_speed'),
            'default_speed': LaunchConfiguration('default_speed'),
            'rotation_speed': LaunchConfiguration('rotation_speed'),
            'scan_angle_range': LaunchConfiguration('scan_angle_range'),
            'depth_obstacle_threshold': 1000,  # mm
            'command_timeout': 2.0  # seconds
        }],
        remappings=[
            ('/scan', '/scan'),
            ('/oak/rgb/image_raw', '/oak/rgb/image_raw'),
            ('/oak/depth/image_raw', '/oak/depth/image_raw'),
            ('/oak/points', '/oak/points')
        ]
    )
    
    # Startup message
    startup_message = LogInfo(
        msg="=== Autonomous Navigation System Starting ==="
    )
    
    safety_message = LogInfo(
        msg="SAFETY: Emergency stop service available at '/emergency_stop'"
    )
    
    control_message = LogInfo(
        msg="CONTROL: Toggle autonomous mode with service '/set_autonomous_mode'"
    )
    
    return LaunchDescription([
        # Launch arguments
        micropython_ip_arg,
        micropython_port_arg,
        autonomous_enabled_arg,
        min_obstacle_distance_arg,
        max_speed_arg,
        default_speed_arg,
        rotation_speed_arg,
        scan_angle_range_arg,
        
        # Startup messages
        startup_message,
        safety_message,
        control_message,
        
        # Launch sensor system
        sensor_launch,
        
        # Launch autonomous navigation
        autonomous_nav_node,
    ])
