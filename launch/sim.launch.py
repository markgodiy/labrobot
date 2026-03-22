import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('labrobot'))
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    # Create a robot_state_publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]
    )

    # Launch Gazebo (gz sim) with default world (ground plane and sun)
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/worlds/empty.sdf'],
        output='screen'
    )

    # Spawn robot in Gazebo with initial z offset to ensure above ground
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'labrobot', '-z', '0.1'],
        output='screen'
    )

    # Static transform publisher for map->odom (if needed for navigation)
    static_tf_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Bridge for common topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',  # Changed to bidirectional
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'
        ],
        output='screen'
    )

    # Separate bridge for joint states with remapping
    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/empty/model/labrobot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        remappings=[
            ('/world/empty/model/labrobot/joint_state', '/joint_states')
        ],
        output='screen'
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time if true'),  # Changed to true for simulation
        node_robot_state_publisher,
        # Note: joint_state_publisher removed for simulation - Gazebo publishes joint states via DiffDrive plugin
        static_tf_pub,
        gazebo,
        bridge,
        joint_state_bridge,  # Added separate joint state bridge
        spawn_entity,
        # Note: rplidar_node removed for simulation - using simulated LIDAR instead
    ])
