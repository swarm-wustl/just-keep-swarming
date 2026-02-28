import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join("/opt/ros/humble/share/depthai_ros_driver", "launch", "camera.launch.py")
            ),
            launch_arguments={
                "name": "oak",
                "parent_frame": "base_link",
                "cam_pos_x": "0.1", # where the camera is on the chassis
                "cam_pos_y": "0.0",
                "cam_pos_z": "0.1",
                "camera.i_enable_imu": "false", 
                "rgb.i_resolution": "1080p",
                "stereo.i_align_depth": "true", 
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')
            ),
            launch_arguments={
                'rtabmap_args': '--delete_db_on_start',
                'rgb_topic': '/oak/rgb/image_raw',
                'depth_topic': '/oak/stereo/image_raw',
                'camera_info_topic': '/oak/rgb/camera_info',
                'frame_id': 'base_link', 
                'approx_sync': 'true',
                'approx_sync_max_interval': '0.1',
                'queue_size': '20',
            }.items(),
        ),
    ])