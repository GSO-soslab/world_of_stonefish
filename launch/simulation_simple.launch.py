import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    wos_param_path = os.path.join(
        get_package_share_directory('world_of_stonefish'),
        'config',
        'params.yaml'
        )
    world_of_stonefish_dir = get_package_share_directory('world_of_stonefish')
    mvp_utility_param_path = os.path.join(
        get_package_share_directory('mvp_utility'),
        'params', 'imu_ned_enu.yaml'
        )


    return LaunchDescription([
        Node(
            package="stonefish_mvp2",
            executable="stonefish_simulator",
            name="stonefish_simulator",
            output="screen",
            parameters= [
                {'data_path': os.path.join(world_of_stonefish_dir, 'data/')},
                {'scenario_path': os.path.join(world_of_stonefish_dir, 'world', 'test.scn') },
                wos_param_path
            ],
        )


])