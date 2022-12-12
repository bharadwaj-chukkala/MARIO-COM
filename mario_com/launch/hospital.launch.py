import os

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file_name = 'simple_hospital.world'
    world = os.path.join(get_package_share_directory('mario_com'),
                         'worlds', world_file_name)
    launch_file_dir = os.path.join(get_package_share_directory('mario_com'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')


    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        # ExecuteProcess(
        #     cmd=['gazebo', '--verbose', world,
        #          '-s', 'libgazebo_ros_factory.so'],
        #     output='screen'),
        
        # ExecuteProcess(
        #     cmd=['ros2', 'param', 'set', '/gazebo',
        #          'use_sim_time', use_sim_time],
        #     output='screen'),
    ])
