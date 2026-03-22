#!/usr/bin/env python3
"""
Lab Robot — Nav2 Navigation Launch
====================================

Launches the full navigation stack using a pre-built map (AMCL localisation +
Nav2 path planning/control).  Use pi.lab.mapping.launch.py first to create
the map, then switch to this file for normal operation.

Nodes launched:
  - robot_state_publisher   (URDF / TF tree)
  - joint_state_publisher
  - sllidar_node            (LIDAR sensor)
  - serial_motor_bridge     (Pico W motor controller)
  - map_server              (serves the saved map)
  - amcl                    (localisation — publishes map→odom TF)
  - controller_server       (DWB local planner)
  - planner_server          (NavFn global planner)
  - behavior_server         (spin, backup, wait)
  - bt_navigator            (action server — NavigateToPose)
  - smoother_server
  - waypoint_follower
  - velocity_smoother
  - lifecycle_manager       (manages all Nav2 nodes)
  - robot_dashboard         (web UI at :8080)

NOTE: No static map→odom TF is published here.
      AMCL provides it dynamically once localised.

Usage:
  ros2 launch labrobot pi.lab.nav.launch.py map:=/home/botmanager/lab_ws/src/labrobot/maps/lab_map.yaml

  Optional args:
    serial_port:=<port>         Pico W serial port (default /dev/ttyACM0)
    enable_rviz:=true           Launch RViz for monitoring (default false)

Send a navigation goal (after AMCL converges):
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\
    "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.5}, orientation: {w: 1.0}}}}"
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('labrobot')
    nav2_params_file = os.path.join(pkg, 'config', 'nav2_params.yaml')

    # --- Launch arguments ---
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(os.path.expanduser('~'), 'lab_ws', 'src', 'labrobot', 'maps', 'lab_map.yaml'),
        description='Full path to the map YAML file',
    )
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyACM0',
        description='Serial port for Pico W motor controller',
    )
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz', default_value='false',
        description='Launch RViz for monitoring',
    )

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

    # --- Serial motor bridge ---
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

    # --- Nav2 nodes ---
    # map_server: serves the saved occupancy grid map
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[
            nav2_params_file,
            {'yaml_filename': LaunchConfiguration('map'), 'use_sim_time': False},
        ],
    )

    # AMCL: Monte Carlo localisation — provides map→odom TF
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[nav2_params_file],
    )

    # Nav2 core nodes
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[nav2_params_file],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        parameters=[nav2_params_file],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        parameters=[nav2_params_file],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=[nav2_params_file],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        parameters=[nav2_params_file],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        parameters=[nav2_params_file],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        parameters=[nav2_params_file],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel'),
        ],
    )

    # Lifecycle manager — starts and transitions all Nav2 nodes
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl',
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
            ],
        }],
    )

    # --- Dashboard ---
    robot_dashboard = Node(
        package='labrobot',
        executable='robot_dashboard.py',
        name='robot_dashboard',
        parameters=[{'use_sim_time': False}],
    )

    # --- Optional RViz ---
    rviz_config = os.path.join(pkg, 'config', 'debug_robot.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('enable_rviz')),
    )

    return LaunchDescription([
        map_arg,
        serial_port_arg,
        enable_rviz_arg,
        robot_state_publisher,
        joint_state_publisher,
        sllidar,
        serial_motor_bridge,
        map_server,
        amcl,
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager,
        robot_dashboard,
        rviz,
    ])
