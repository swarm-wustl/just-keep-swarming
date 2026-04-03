import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node # <--- ADD THIS
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    return LaunchDescription([
        
        # 1. Your OAK-D Launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join("/opt/ros/humble/share/depthai_ros_driver", "launch", "rgbd_pcl.launch.py")
            ),
            launch_arguments={
                "name": "oak",
                "parent_frame": "base_link",
                "cam_pos_x": "0.0", 
                "cam_pos_y": "0.0",
                "cam_pos_z": "0.1",
                "camera.i_enable_imu": "true", 
                "rgb.i_resolution": "1080p",
                "stereo.i_align_depth": "true",
                "use_rviz": "false" 
            }.items(),
        ),

        # 2. ADD THE IMU FILTER NODE
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            parameters=[{
                'use_mag': False,
                'publish_tf': False,
                'world_frame': 'enu',
            }],
            remappings=[
                ('/imu/data_raw', '/oak/imu/data'),
                ('/imu/data', '/oak/imu/data_filtered') 
            ]
        ),

        # 3. Your RTAB-Map Launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')
            ),
            launch_arguments={
                'rtabmap_args': '--delete_db_on_start',
                'wait_imu_to_init': 'true',
                'rgb_topic': '/oak/rgb/image_raw',
                'depth_topic': '/oak/stereo/image_raw',
                'camera_info_topic': '/oak/rgb/camera_info',
                'imu_topic': '/oak/imu/data_filtered',
                'qos_imu': '2', 
                'frame_id': 'base_link', 
                'approx_sync': 'true',
                'approx_sync_max_interval': '0.02', 
                'queue_size': '20',
                'cloud_noise_filtering_min_neighbors': '10',
                'cloud_noise_filtering_radius': '0.5'
            }.items(),
        ),
    ])

