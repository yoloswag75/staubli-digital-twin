import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('staubli_description')

    urdf_file = os.path.join(pkg, 'urdf', 'rx160l.urdf')
    rviz_file = os.path.join(pkg, 'rviz', 'rx160l.rviz')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        # robot_state_publisher : lit l'URDF + /joint_states → publie les TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # RViz2 avec config préconfigurée
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file]
        ),
    ])
