import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    world = os.path.join(get_package_share_directory('mario_com'),
                         'worlds', 'small_hospital.world')
    
    model = os.path.join(get_package_share_directory('mario_com'),
                         'models')

    map = os.path.join(get_package_share_directory('mario_com'),
                         'maps', 'hospitalmap.yaml')
    print(map)
    launch_file_dir = os.path.join(get_package_share_directory('mario_com'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    tb3_man_bgp = get_package_share_directory('turtlebot3_manipulation_bringup')
    nav2_man = get_package_share_directory('turtlebot3_manipulation_navigation2')

    included_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([
                tb3_man_bgp + '/launch/gazebo.launch.py']),
                launch_arguments={'world': world, 'x_pose': '0.0', 'y_pose': '0.0'}.items())

    bins_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([
                launch_file_dir, '/bins.launch.py']))

    nav_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([
                nav2_man + '/launch/navigation2.launch.py']),
                launch_arguments={'map_yaml_file': map, 'start_rviz': 'True', 'params_file': nav2_man+'/param/turtlebot3_use_sim_time.yaml'}.items())

    initial_pose_pub = ExecuteProcess(
        cmd=[
            'ros2', 'topic pub -1', '/initialpose', 'geometry_msgs/PoseWithCovarianceStamped', '"{ header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: { pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, } }"' 
        ],
        shell=True
        )

    return LaunchDescription([
        included_launch,
        bins_launch,
        nav_launch,
        initial_pose_pub,
    ])
        