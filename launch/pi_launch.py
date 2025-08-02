import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
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

    # Create a joint_state_publisher node (non-GUI) - for real robot without simulation
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Static transform publisher for map->odom (if needed for navigation)
    static_tf_pub_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Static transform publisher for odom->base_link (basic odometry for real robot)
    # Note: In a real robot setup, this should be replaced with actual odometry from wheel encoders
    static_tf_pub_odom_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # RPLIDAR node configuration
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_f4ff8904fb63ef118309e1a9c169b110-if00-port0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='460800')
    frame_id = LaunchConfiguration('frame_id', default='lidar_frame')  # Use your robot's frame
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Standard')

    # Note: LIDAR will fail to start if not connected - this is expected for testing without hardware
    rplidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[
            {'channel_type': channel_type},
            {'serial_port': serial_port},
            {'serial_baudrate': serial_baudrate},
            {'frame_id': frame_id},
            {'inverted': inverted},
            {'angle_compensate': angle_compensate},
            {'scan_mode': scan_mode}
        ],
        output='screen'
    )

    # OAK-D Lite Camera configuration (ENABLED - power issue resolved!)
    oak_camera = Node(
        package='depthai_ros_driver',
        executable='camera_node',  # Correct executable name
        name='oak_camera',
        parameters=[{
            'camera_model': 'OAK-D-LITE',
            'tf_prefix': 'oak_camera',  # Use oak_camera prefix to match actual topics
            'mode': 'depth',  # Full depth mode now that power is stable
            'depth_enabled': True,  # Enable depth for navigation/obstacle avoidance
            'stereo_enabled': True,
            'rgb_enabled': True,
            'pointcloud_enabled': False,  # Disable built-in, use RGB overlay version
            'use_sim_time': use_sim_time,
            # Quality settings
            'rgb_resolution': '720p',
            'depth_resolution': '720p',
            'fps': 15,  # Reasonable FPS for Pi performance
            'publish_tf_from_calibration': True,
            # Let depthai use its default topic naming
        }],
        output='screen'
    )

    # RGB Point Cloud Processing Container for RGB overlay on point cloud
    # This creates colored point clouds by combining RGB and depth images
    point_cloud_container = ComposableNodeContainer(
        name='point_cloud_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzrgbNode',
                name='point_cloud_xyzrgb_node',
                remappings=[
                    # Map OAK-D camera topics to standard depth_image_proc inputs
                    ('rgb/image_rect_color', '/oak_camera/rgb/image_raw'),
                    ('rgb/camera_info', '/oak_camera/rgb/camera_info'),
                    ('depth_registered/image_rect', '/oak_camera/stereo/image_raw'),
                    ('depth_registered/camera_info', '/oak_camera/stereo/camera_info'),
                    # Output colored point cloud
                    ('points', '/oak_camera/points_xyzrgb')
                ],
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'queue_size': 5,  # Synchronization queue size
                    'approximate_sync': True,  # Use approximate time sync for better performance
                }]
            ),
        ],
        output='screen',
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        DeclareLaunchArgument('channel_type', default_value=channel_type, description='Specifying channel type of lidar'),
        DeclareLaunchArgument('serial_port', default_value=serial_port, description='Specifying usb port to connected lidar'),
        DeclareLaunchArgument('serial_baudrate', default_value=serial_baudrate, description='Specifying usb port baudrate to connected lidar'),
        DeclareLaunchArgument('frame_id', default_value=frame_id, description='Specifying frame_id of lidar'),
        DeclareLaunchArgument('inverted', default_value=inverted, description='Specifying whether or not to invert scan data'),
        DeclareLaunchArgument('angle_compensate', default_value=angle_compensate, description='Specifying whether or not to enable angle_compensate of scan data'),
        DeclareLaunchArgument('scan_mode', default_value=scan_mode, description='Specifying scan mode of lidar'),

        node_robot_state_publisher,
        node_joint_state_publisher,
        static_tf_pub_map_odom,
        static_tf_pub_odom_base,
        rplidar_node,
        oak_camera,  # ENABLED - power issue resolved!
        point_cloud_container,  # RGB Point Cloud Processing
        # Note: No ROS-Gazebo bridge needed for Raspberry Pi hardware
    ])
