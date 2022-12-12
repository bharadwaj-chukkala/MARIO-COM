import os
import random

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    x_bin = random.randint(-5, 5)
    y_bin = random.randint(-2, 3)

    model = os.path.join(get_package_share_directory('mario_com'),
                         'models')

    spawn_bin_1 = Node(
    package='gazebo_ros', 
    executable='spawn_entity.py',
    arguments=['-entity', 'trash_bin1', 
            '-file', model + '/aws_robomaker_residential_Trash_01/model.sdf',
            '-x', str(x_bin),
            '-y', str(y_bin),
            '-z', '0.0',
            '-Y', '0.0'],
            output='screen')

    spawn_bin_2 = Node(
    package='gazebo_ros', 
    executable='spawn_entity.py',
    arguments=['-entity', 'trash_bin2', 
            '-file', model + '/aws_robomaker_residential_Trash_01/model.sdf',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-Y', '0.0'],
            output='screen')

    return LaunchDescription([
        spawn_bin_1,
        spawn_bin_2,
    ])
        