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
                         'worlds', 'simple_hospital.world')
    
    model = os.path.join(get_package_share_directory('mario_com'),
                         'models')

    launch_file_dir = os.path.join(get_package_share_directory('mario_com'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    tb3_man_bgp = get_package_share_directory('turtlebot3_manipulation_bringup')
    included_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([
                tb3_man_bgp + '/launch/gazebo.launch.py']),
                launch_arguments={'world': world, 'x_pose': '-7.0', 'y_pose': '4.0'}.items())

    bins_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([
                launch_file_dir, '/bins.launch.py']))


    return LaunchDescription([
        # SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='waffle_pi'), 
        # SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=model), 
        included_launch,
        # bins_launch,
    ])
        