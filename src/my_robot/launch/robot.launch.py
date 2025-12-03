from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot')
    urdf_path = os.path.join(pkg_share, 'model', 'Differential_bot.urdf')
    world_path = os.path.join(os.path.expanduser('~'), '10X_task', 'worlds', 'maze.world')

    with open(urdf_path, 'r') as file:
        robot_desc = file.read()

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_cam_base",
        arguments=["0", "0", "0", "0", "0", "0", "camera_link", "base_link"]
    )

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                )
            ),
            launch_arguments={
                'verbose': 'true',
                'world': world_path
            }.items()
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_robot',
            output='screen',
            arguments=[
                '-file', urdf_path,
                '-entity', 'diff_robot',
                '-x', '-9', '-y', '3', '-z', '0.1'
            ]
        ),

        static_tf,

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge',
            output='screen',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'
            ]
        )
    ])
