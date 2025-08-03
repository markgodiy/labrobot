import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import xacro


def generate_launch_description():
    """
    Full Pi launcher - Robot model with ALL sensors: LIDAR + OAK-D Lite + IMU
    Complete robot perception suite optimized for computer vision, SLAM, navigation, and obstacle avoidance
    Includes high-quality streaming, point cloud processing, and comprehensive sensor integration
    """
    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('labrobot'))
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file, mappings={'use_sim_time': 'false'})
    
    # Create a robot_state_publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config.toxml(),
            'use_sim_time': use_sim_time
        }]
    )

    # Create a joint_state_publisher node (non-GUI) - for real robot
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config.toxml(),
            'use_sim_time': use_sim_time
        }]
    )

    # Static transform publisher for map->odom (for RViz reference frame)
    static_tf_pub_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1', '--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Static transform publisher for odom->base_footprint (basic odometry)
    static_tf_pub_odom_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_footprint_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1', '--frame-id', 'odom', '--child-frame-id', 'base_footprint'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Static transform publisher to connect OAK-D base frame to robot's depth camera frame
    # This ensures the OAK-D TF tree is properly connected to the robot
    static_tf_pub_oak_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='oak_to_depth_camera_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1', '--frame-id', 'depth_camera_link', '--child-frame-id', 'oak'],
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

    # RPLIDAR node - requires hardware to be connected
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
            {'scan_mode': scan_mode},
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # OAK-D Lite Camera configuration - Optimized for computer vision and streaming
    oak_camera = Node(
        package='depthai_ros_driver',
        executable='camera_node',
        name='oak',
        parameters=[{
            # Basic camera configuration
            'camera.i_pipeline_type': 'RGBD',
            'camera.i_publish_tf_from_calibration': True,
            'camera.i_tf_base_frame': 'oak',
            'camera.i_tf_camera_name': 'oak',
            'camera.i_tf_parent_frame': 'depth_camera_link',  # Link to robot URDF depth_camera_link
            'use_sim_time': use_sim_time,
            
            # RGB sensor configuration - Higher quality for CV
            'rgb.i_publish_topic': True,
            'rgb.i_fps': 30.0,  # Higher FPS for smooth streaming
            'rgb.i_resolution': '1080P',  # Higher resolution for better CV
            'rgb.i_output_disparity': False,
            'rgb.i_get_base_device_timestamp': True,
            
            # Stereo/Depth configuration - Optimized for point cloud quality
            'stereo.i_publish_topic': True,
            'stereo.i_fps': 30.0,
            'stereo.i_resolution': '720P',
            'stereo.i_align_depth': True,
            'stereo.i_output_disparity': True,  # Enable disparity for advanced CV
            'stereo.i_get_base_device_timestamp': True,
            
            # Left/Right camera topics - Useful for stereo vision
            'left.i_publish_topic': True,
            'left.i_fps': 30.0,
            'left.i_resolution': '720P',
            'right.i_publish_topic': True,
            'right.i_fps': 30.0,
            'right.i_resolution': '720P',
            
            # IMU for motion tracking
            'imu.i_publish_topic': True,
            'pipeline_gen.i_enable_imu': True,
            'imu.i_get_base_device_timestamp': True,
            
            # Quality settings for CV applications
            'rgb.i_low_bandwidth': False,
            'stereo.i_low_bandwidth': False,
            'rgb.i_keep_preview_aspect_ratio': True,
        }],
        output='screen'
    )

    # Computer Vision Processing Container
    cv_processing_container = ComposableNodeContainer(
        name='cv_processing_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # High-quality XYZ point cloud
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzNode',
                name='point_cloud_xyz_node',
                remappings=[
                    ('image_rect', '/oak/stereo/image_raw'),
                    ('camera_info', '/oak/stereo/camera_info'),
                    ('points', '/oak/points')
                ],
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'approximate_sync': True,
                    'queue_size': 10,
                }]
            ),
            # RGB overlay point cloud for colored visualization
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzrgbNode',
                name='point_cloud_xyzrgb_node',
                remappings=[
                    ('rgb/image_rect_color', '/oak/rgb/image_raw'),
                    ('rgb/camera_info', '/oak/rgb/camera_info'),
                    ('depth_registered/image_rect', '/oak/stereo/image_raw'),
                    ('depth_registered/camera_info', '/oak/stereo/camera_info'),
                    ('points', '/oak/points_xyzrgb')
                ],
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'approximate_sync': True,
                    'queue_size': 10,
                }]
            ),
            # Rectify RGB image for better CV processing
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rgb_rectify_node',
                remappings=[
                    ('image', '/oak/rgb/image_raw'),
                    ('camera_info', '/oak/rgb/camera_info'),
                    ('image_rect', '/oak/rgb/image_rect')
                ],
                parameters=[{
                    'use_sim_time': use_sim_time,
                }]
            ),
            # Rectify depth image
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='depth_rectify_node',
                remappings=[
                    ('image', '/oak/stereo/image_raw'),
                    ('camera_info', '/oak/stereo/camera_info'),
                    ('image_rect', '/oak/stereo/image_rect')
                ],
                parameters=[{
                    'use_sim_time': use_sim_time,
                }]
            ),
        ],
        output='screen',
    )

    # Image transport republishers for different compression formats and easier RViz access
    rgb_compressed_publisher = Node(
        package='image_transport',
        executable='republish',
        name='rgb_compressed_publisher',
        arguments=['raw', 'compressed'],
        remappings=[
            ('in', '/oak/rgb/image_raw'),
            ('out', '/oak/rgb/compressed')
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
            'compressed.jpeg_quality': 90,  # High quality for CV
        }]
    )

    depth_compressed_publisher = Node(
        package='image_transport',
        executable='republish',
        name='depth_compressed_publisher',
        arguments=['raw', 'compressedDepth'],
        remappings=[
            ('in', '/oak/stereo/image_raw'),
            ('out', '/oak/stereo/compressed')
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
            'compressedDepth.depth_max': 10.0,  # 10 meter max depth
            'compressedDepth.depth_quantization': 100.0,  # Good precision
        }]
    )

    # Standard topic republishers for compatibility
    rgb_republisher = Node(
        package='image_transport',
        executable='republish',
        name='rgb_republisher',
        arguments=['raw'],
        remappings=[
            ('in', '/oak/rgb/image_raw'),
            ('out', '/camera/rgb/image_raw')
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    depth_republisher = Node(
        package='image_transport',
        executable='republish',
        name='depth_republisher',
        arguments=['raw'],
        remappings=[
            ('in', '/oak/stereo/image_raw'),
            ('out', '/camera/depth/image_raw')
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Note: Camera info and IMU topics available directly from OAK-D Lite:
    # - /oak/rgb/camera_info (RGB camera info)
    # - /oak/stereo/camera_info (Depth camera info) 
    # - /oak/imu/data (IMU data)
    # Standard republished topics via image_transport already available above

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        # LIDAR launch arguments
        DeclareLaunchArgument('channel_type', default_value=channel_type, description='Specifying channel type of lidar'),
        DeclareLaunchArgument('serial_port', default_value=serial_port, description='Specifying usb port to connected lidar'), 
        DeclareLaunchArgument('serial_baudrate', default_value=serial_baudrate, description='Specifying usb port baudrate to connected lidar'),
        DeclareLaunchArgument('frame_id', default_value=frame_id, description='Specifying frame_id of lidar'),
        DeclareLaunchArgument('inverted', default_value=inverted, description='Specifying whether or not to invert scan data'),
        DeclareLaunchArgument('angle_compensate', default_value=angle_compensate, description='Specifying whether or not to enable angle_compensate of scan data'),
        DeclareLaunchArgument('scan_mode', default_value=scan_mode, description='Specifying scan mode of lidar'),

        # Core robot nodes
        node_robot_state_publisher,
        node_joint_state_publisher,
        static_tf_pub_map_odom,
        static_tf_pub_odom_base,
        static_tf_pub_oak_base,  # Connect OAK-D TF tree to robot
        
        # Sensor nodes
        rplidar_node,  # LIDAR for SLAM and navigation
        oak_camera,  # OAK-D Lite with CV-optimized settings
        cv_processing_container,  # Point clouds and image processing
        
        # Image streaming and compression
        rgb_compressed_publisher,  # Compressed RGB for bandwidth efficiency
        depth_compressed_publisher,  # Compressed depth
        rgb_republisher,  # Standard RGB topic
        depth_republisher,  # Standard depth topic
        
        # Note: Direct topics available from OAK-D Lite:
        # - /oak/rgb/camera_info & /oak/stereo/camera_info (camera info) 
        # - /oak/imu/data (IMU data)
        # - /scan (LIDAR data)
        # - Optimized for full sensor suite: computer vision, SLAM, navigation, and streaming
    ])
