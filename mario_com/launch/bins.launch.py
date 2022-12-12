import os
import random

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_random_bins(num_bins):

    x_bins = random.sample(range(0, 2), num_bins)
    y_bins = random.sample(range(1, 7), num_bins)
    
    return x_bins, y_bins

def gen_node(idx, xpos, ypos):
    model = os.path.join(get_package_share_directory('mario_com'),
                         'models')
    
    return Node(
    package='gazebo_ros', 
    executable='spawn_entity.py',
    arguments=['-entity', 'trash_bin{}'.format(idx), 
            '-file', model + '/bin_cylinder/model.sdf',
            '-x', str(xpos),
            '-y', str(ypos),
            '-z', '0.0',
            '-Y', '0.0'],
            output='screen')

def generate_launch_description():
    
    num_bins = 1
    x_bins, y_bins = generate_random_bins(num_bins)
    nodes = []
    for i in range(num_bins):
        nodes.append(gen_node(i+1, x_bins[i], y_bins[i]))

    model = os.path.join(get_package_share_directory('mario_com'),
                         'models')

    return LaunchDescription([
        nodes[0],
    ])
        