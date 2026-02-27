from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Camera Launch
    depthai_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('depthai_ros_driver'),
                'launch',
                'rgbd_pcl.launch.py'
            ])
        )
    )

    # 
    odom_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        parameters=[{
            'frame_id': 'oak',
            'approx_sync': True,
            'subscribe_depth': True,
            'subscribe_rgb': True,
        }],
        remappings=[
            ('rgb/image', '/oak/rgb/image_raw'),
            ('depth/image', '/oak/stereo/image_raw'),
            ('rgb/camera_info', '/oak/rgb/camera_info'),
            ('odom', '/odom')
        ],
        output='screen'
    )

    # 3. Launch RTAB-Map with internal OctoMap generation
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        parameters=[{
            'frame_id': 'oak',
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'approx_sync': True,
            'queue_size': 30,               # Replaced sync_queue_size to match rtabmap_slam conventions
            
            #OctoMap Parameters
            'Grid/3D': 'true',              # Forces RTAB-Map to build a 3D voxel map
            'Grid/RayTracing': 'true',      # Clears empty space automatically behind the camera
            'Grid/CellSize': '0.05',        # 5cm voxels 
            'Grid/RangeMax': '5.0',         # Ignore noisy depth points past 5 meters
        }],
        remappings=[
            ('rgb/image', '/oak/rgb/image_raw'),
            ('depth/image', '/oak/stereo/image_raw'),
            ('rgb/camera_info', '/oak/rgb/camera_info'),
            ('odom', '/odom'),
        ],
        arguments=['--delete_db_on_start']  # Clear memory on startup for clean testing
    )

    return LaunchDescription([
        depthai_launch,
        odom_node,
        rtabmap_node
    ])