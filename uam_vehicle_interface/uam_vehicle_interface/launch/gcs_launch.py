import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    vehicle_interface_dir = get_package_share_directory('uam_vehicle_interface')
    launch_dir = os.path.join(vehicle_interface_dir, 'launch')

    params_file = os.path.join(vehicle_interface_dir, 'params', 'vehicle_params.yaml')

    rviz2_node = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + os.path.join(vehicle_interface_dir, 'rviz', 'config_file.rviz')]
    )

    ld = LaunchDescription()

    ld.add_action(rviz2_node)

    return ld