import os
import socket
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import xacro


def generate_launch_description():
    """
    Pi CV launcher - Basic sensors + OAK-D Lite depth camera, with Fast DDS discovery server enabled (unicast)
    Suitable for computer vision and perception tasks with improved DDS discovery for multi-robot or remote operation
    """
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('labrobot'))
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file, mappings={'use_sim_time': 'false'})

    # Automatically detect the device's primary IP address for ROS_DISCOVERY_SERVER
    def get_primary_ip():
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            # Doesn't have to be reachable
            s.connect(('8.8.8.8', 80))
            ip = s.getsockname()[0]
        except Exception:
            ip = '127.0.0.1'
        finally:
            s.close()
        return ip

    discovery_server_ip = get_primary_ip()
    discovery_server_env = {
        'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
        'FASTRTPS_DEFAULT_PROFILES_FILE': os.path.join(pkg_path, 'config', 'fastrtps_discovery_server_unicast.xml'),
        'ROS_DISCOVERY_SERVER': f'{discovery_server_ip}:11811'
    }

    # Robot state publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config.toxml(),
            'use_sim_time': use_sim_time
        }],
        additional_env=discovery_server_env
    )

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

    static_tf_pub_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1', '--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    static_tf_pub_odom_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_footprint_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1', '--frame-id', 'odom', '--child-frame-id', 'base_footprint'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    static_tf_pub_oak_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='oak_to_depth_camera_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1', '--frame-id', 'depth_camera_link', '--child-frame-id', 'oak'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_f4ff8904fb63ef118309e1a9c169b110-if00-port0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='460800')
    frame_id = LaunchConfiguration('frame_id', default='lidar_frame')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Standard')

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

    oak_camera = Node(
        package='depthai_ros_driver',
        executable='camera_node',
        name='oak',
        parameters=[{
            'camera.i_pipeline_type': 'RGBD',
            'camera.i_publish_tf_from_calibration': True,
            'camera.i_tf_base_frame': 'oak',
            'camera.i_tf_camera_name': 'oak',
            'camera.i_tf_parent_frame': 'depth_camera_link',
            'use_sim_time': use_sim_time,
            'rgb.i_publish_topic': True,
            'rgb.i_fps': 15.0,
            'rgb.i_resolution': '720P',
            'stereo.i_publish_topic': True,
            'stereo.i_fps': 15.0,
            'stereo.i_resolution': '720P',
            'stereo.i_align_depth': True,
            'left.i_publish_topic': False,
            'right.i_publish_topic': False,
            'imu.i_publish_topic': True,
            'pipeline_gen.i_enable_imu': True,
            'rgb.i_low_bandwidth': False,
            'stereo.i_low_bandwidth': False,
        }],
        output='screen'
    )

    point_cloud_container = ComposableNodeContainer(
        name='point_cloud_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzNode',
                name='point_cloud_xyz_node',
                remappings=[
                    ('image_rect', '/oak/stereo/image_raw'),
                    ('points', '/oak/points_xyz')
                ],
                parameters=[{
                    'use_sim_time': use_sim_time,
                }]
            ),
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzrgbNode',
                name='point_cloud_xyzrgb_node',
                remappings=[
                    ('rgb/image_rect_color', '/oak/rgb/image_raw'),
                    ('depth_registered/image_rect', '/oak/stereo/image_raw'),
                    ('points', '/oak/points_xyzrgb')
                ],
                parameters=[{
                    'use_sim_time': use_sim_time,
                }]
            ),
        ],
        output='screen',
    )

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
        static_tf_pub_oak_base,
        rplidar_node,
        oak_camera,
        point_cloud_container,
        rgb_republisher,
        depth_republisher,
    ])
