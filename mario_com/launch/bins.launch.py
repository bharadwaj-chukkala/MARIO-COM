import os
import random

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_random_bins(num_bins):

    x_bins = random.sample(range(-5, 5), num_bins)
    y_bins = random.sample(range(-2, 3), num_bins)

    print("X bins")
    print(x_bins)
    print("Y bins")
    print(y_bins)

    # x_pos = []
    # y_pos = []
    # # Generate x position for bins
    # while True:
    #     if len(x_pos)==number:
    #         break
        
    #     x_bin = random.randint(-5, 5)

    #     if (len(x_pos)==0) or (x_bin not in x_pos):
    #         x_pos.append(x_bin)

    
    # # Generate y position for bins
    # while True:
    #     if len(y_pos)==number:
    #         break
        
    #     y_bin = random.randint(-2, 3)

    #     if (len(y_pos)==0) or (y_bin not in y_pos):
    #         y_pos.append(y_bin)
    
    return x_bins, y_bins#x_pos,y_pos

def gen_node(idx, xpos, ypos):
    model = os.path.join(get_package_share_directory('mario_com'),
                         'models')
    return Node(
    package='gazebo_ros', 
    executable='spawn_entity.py',
    arguments=['-entity', 'trash_bin{}'.format(idx), 
            '-file', model + '/aws_robomaker_residential_Trash_01/model.sdf',
            '-x', str(xpos),
            '-y', str(ypos),
            '-z', '0.0',
            '-Y', '0.0'],
            output='screen')

def generate_launch_description():
    
    num_bins = 2
    x_bins, y_bins = generate_random_bins(num_bins)
    nodes = []
    for i in range(num_bins):
        print("fhedtghd")
        nodes.append(gen_node(i+1, x_bins[i], y_bins[i]))

    model = os.path.join(get_package_share_directory('mario_com'),
                         'models')

    # spawn_bin_1 = Node(
    # package='gazebo_ros', 
    # executable='spawn_entity.py',
    # arguments=['-entity', 'trash_bin1', 
    #         '-file', model + '/aws_robomaker_residential_Trash_01/model.sdf',
    #         '-x', str(x_bins[0]),
    #         '-y', str(y_bins[0]),
    #         '-z', '0.0',
    #         '-Y', '0.0'],
    #         output='screen')

    # spawn_bin_2 = Node(
    # package='gazebo_ros', 
    # executable='spawn_entity.py',
    # arguments=['-entity', 'trash_bin2', 
    #         '-file', model + '/aws_robomaker_residential_Trash_01/model.sdf',
    #         '-x', str(x_bins[1]),
    #         '-y', str(y_bins[1]),
    #         '-z', '0.0',
    #         '-Y', '0.0'],
    #         output='screen')

    return LaunchDescription([
        nodes[0],
        nodes[1],
        # spawn_bin_1,
        # spawn_bin_2,
    ])
        