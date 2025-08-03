#!/usr/bin/env python3
"""
Serial-based Autonomous Navigation Launch File
==============================================

Launches the complete autonomous navigation system using serial communication:
- Robot model and sensors (LIDAR + OAK-D Lite)
- Serial motor bridge for MicroPython communication
- Autonomous navigation node

Usage:
  ros2 launch labrobot pi.autonomous.serial.launch.py

Parameters:
  - serial_port: Serial port for MicroPython controller (default: /dev/ttyACM0)
  - baudrate: Serial baudrate (default: 115200)
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

def generate_launch_description():
    # Package directory
    pkg_share = FindPackageShare('labrobot')
    
    # Launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for MicroPython motor controller'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Serial baudrate for communication'
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
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Start RViz for visualization'
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
            'use_rviz': LaunchConfiguration('use_rviz')
        }.items()
    )
    
    # Serial motor bridge node
    serial_bridge_node = Node(
        package='labrobot',
        executable='serial_motor_bridge.py',
        name='serial_motor_bridge',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'timeout': 1.0,
            'reconnect_interval': 5.0
        }],
        remappings=[
            ('/cmd_vel', '/cmd_vel')
        ]
    )
    
    # Autonomous navigation node
    autonomous_nav_node = Node(
        package='labrobot',
        executable='autonomous_navigation_node.py',
        name='autonomous_navigation_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
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
    
    # Startup messages
    startup_message = LogInfo(
        msg="=== Serial-based Autonomous Navigation System Starting ==="
    )
    
    serial_message = LogInfo(
        msg="SERIAL: MicroPython controller communication via USB serial"
    )
    
    safety_message = LogInfo(
        msg="SAFETY: Emergency stop service available at '/emergency_stop'"
    )
    
    control_message = LogInfo(
        msg="CONTROL: Toggle autonomous mode with service '/set_autonomous_mode'"
    )
    
    status_message = LogInfo(
        msg="STATUS: Monitor with 'ros2 topic echo /navigation/state' and '/motor_controller/status'"
    )
    
    return LaunchDescription([
        # Launch arguments
        serial_port_arg,
        baudrate_arg,
        autonomous_enabled_arg,
        min_obstacle_distance_arg,
        max_speed_arg,
        default_speed_arg,
        rotation_speed_arg,
        scan_angle_range_arg,
        use_rviz_arg,
        
        # Startup messages
        startup_message,
        serial_message,
        safety_message,
        control_message,
        status_message,
        
        # Launch sensor system
        sensor_launch,
        
        # Launch serial bridge
        serial_bridge_node,
        
        # Launch autonomous navigation
        autonomous_nav_node,
    ])
