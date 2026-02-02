from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    # 1. Start RealSense Camera
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])
        ),
        launch_arguments={
            'align_depth.enable': 'true',
            'pointcloud.enable': 'false', 
            # Ensure profiles are set (optional but good for stability)
            'depth_module.depth_profile': '848x480x30',
            'rgb_camera.color_profile': '1280x720x30',
        }.items()
    )

    # 2. Start the Mock Python Filter (With Updated QoS)
    mock_filter_node = Node(
        package='camera_checking',
        executable='mock_filter.py',
        name='mock_filter_node',
        output='screen'
    )

    # 3. Start Depth Image Proc
    # It will automatically inherit Best Effort QoS from the mock_filter inputs
    point_cloud_container = ComposableNodeContainer(
        name='depth_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzNode',
                name='point_cloud_xyz_node',
                remappings=[
                    ('image_rect', '/filtered_depth_image'),
                    ('camera_info', '/filtered_depth_camera_info'),
                    ('points', '/points') 
                ],
                # Explicitly Allow Any QoS for input to prevent startup race conditions
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    # 4. RViz - NOTE: You MUST set QoS manually in GUI if this doesn't work automatically
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', '', '-f', 'camera_link'], 
    )

    return LaunchDescription([
        realsense_launch,
        mock_filter_node,
        point_cloud_container,
        rviz_node
    ])