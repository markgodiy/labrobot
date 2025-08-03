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
    Computer Vision (CV) Pi launcher - Robot model with focus on visual perception and streaming
    Optimized for RViz visualization with RGB, depth, point clouds, and image processing
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
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Static transform publisher for odom->base_footprint (basic odometry)
    static_tf_pub_odom_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_footprint_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
        parameters=[{'use_sim_time': use_sim_time}]
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
            'camera.i_tf_parent_frame': 'oak_camera_link',
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
        arguments=['raw', 'compressed', 'in:=/oak/rgb/image_raw', 'out:=/oak/rgb/compressed'],
        parameters=[{
            'use_sim_time': use_sim_time,
            'compressed.jpeg_quality': 90,  # High quality for CV
        }]
    )

    depth_compressed_publisher = Node(
        package='image_transport',
        executable='republish',
        name='depth_compressed_publisher',
        arguments=['raw', 'compressedDepth', 'in:=/oak/stereo/image_raw', 'out:=/oak/stereo/compressed'],
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
        arguments=['raw', 'in:=/oak/rgb/image_raw', 'out:=/camera/rgb/image_raw'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    depth_republisher = Node(
        package='image_transport',
        executable='republish',
        name='depth_republisher',
        arguments=['raw', 'in:=/oak/stereo/image_raw', 'out:=/camera/depth/image_raw'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Camera info republishers
    rgb_info_republisher = Node(
        package='topic_tools',
        executable='relay',
        name='rgb_info_republisher',
        arguments=['/oak/rgb/camera_info', '/camera/rgb/camera_info'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    depth_info_republisher = Node(
        package='topic_tools',
        executable='relay',
        name='depth_info_republisher',
        arguments=['/oak/stereo/camera_info', '/camera/depth/camera_info'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # IMU data republisher for easier RViz access and standard topic naming
    imu_republisher = Node(
        package='topic_tools',
        executable='relay',
        name='imu_republisher',
        arguments=['/oak/imu/data', '/imu/data'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        # Core robot nodes
        node_robot_state_publisher,
        node_joint_state_publisher,
        static_tf_pub_map_odom,
        static_tf_pub_odom_base,
        
        # Computer vision nodes
        oak_camera,  # OAK-D Lite with CV-optimized settings
        cv_processing_container,  # Point clouds and image processing
        
        # Image streaming and compression
        rgb_compressed_publisher,  # Compressed RGB for bandwidth efficiency
        depth_compressed_publisher,  # Compressed depth
        rgb_republisher,  # Standard RGB topic
        depth_republisher,  # Standard depth topic
        rgb_info_republisher,  # RGB camera info
        depth_info_republisher,  # Depth camera info
        imu_republisher,  # IMU data for motion tracking
        
        # Note: Optimized for computer vision, streaming, IMU, and RViz visualization
    ])
