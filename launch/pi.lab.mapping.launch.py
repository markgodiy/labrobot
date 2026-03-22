#!/usr/bin/env python3
"""
Lab Robot — SLAM Mapping Launch
================================

Launches the robot for building a map of the lab using SLAM Toolbox.
Run this once to create the map, then switch to pi.lab.nav.launch.py.

Nodes launched:
  - robot_state_publisher   (URDF / TF tree)
  - joint_state_publisher
  - sllidar_node            (LIDAR sensor)
  - serial_motor_bridge     (Pico W motor controller)
  - async_slam_toolbox_node (online mapping — publishes map→odom TF)

NOTE: No static map→odom TF is published here.
      SLAM Toolbox provides it dynamically.

Usage:
  ros2 launch labrobot pi.lab.mapping.launch.py

Teleop while mapping:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard

Save map when done:
  ros2 run nav2_map_server map_saver_cli -f ~/lab_ws/src/labrobot/maps/lab_map
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('labrobot')

    # --- Robot description ---
    xacro_file = os.path.join(pkg, 'description', 'robot.urdf.xacro')
    robot_desc = xacro.process_file(xacro_file, mappings={'use_sim_time': 'false'}).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': False}],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': False}],
    )

    # --- LIDAR ---
    lidar_port = '/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_f4ff8904fb63ef118309e1a9c169b110-if00-port0'

    sllidar = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': lidar_port,
            'serial_baudrate': 460800,
            'frame_id': 'lidar_frame',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }],
    )

    # --- Serial motor bridge (Pico W) ---
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyACM0',
        description='Serial port for Pico W motor controller',
    )

    serial_motor_bridge = Node(
        package='labrobot',
        executable='serial_motor_bridge.py',
        name='serial_motor_bridge',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': 115200,
            'wheel_base_m': 0.347,
            'wheel_radius_m': 0.0325,
            'ticks_per_rev': 3436,
        }],
    )

    # --- SLAM Toolbox (online async mapping) ---
    slam_params = {
        'use_sim_time': False,
        'solver_plugin': 'solver_plugins::CeresSolver',
        'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY',
        'ceres_preconditioner': 'SCHUR_JACOBI',
        'ceres_trust_strategy': 'LEVENBERG_MARQUARDT',
        'ceres_dogleg_type': 'TRADITIONAL_DOGLEG',
        'ceres_loss_function': 'None',
        'odom_frame': 'odom',
        'map_frame': 'map',
        'base_frame': 'base_footprint',
        'scan_topic': '/scan',
        'use_map_saver': False,
        'mode': 'mapping',
        'debug_logging': False,
        'throttle_scans': 1,
        'transform_publish_period': 0.02,
        'map_update_interval': 5.0,
        'resolution': 0.05,
        'max_laser_range': 12.0,
        'minimum_time_interval': 0.5,
        'transform_timeout': 0.2,
        'tf_buffer_duration': 30.0,
        'stack_size_to_use': 40000000,
        'enable_interactive_mode': False,
    }

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params],
    )

    # Lifecycle manager — configures and activates slam_toolbox automatically
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['slam_toolbox'],
            'bond_timeout': 15.0,  # slam_toolbox init takes >4s default timeout
        }],
    )

    return LaunchDescription([
        serial_port_arg,
        robot_state_publisher,
        joint_state_publisher,
        sllidar,
        serial_motor_bridge,
        slam_toolbox,
        lifecycle_manager,
    ])
