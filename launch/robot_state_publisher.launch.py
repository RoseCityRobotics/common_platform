import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'common_platform'
    file_subpath = 'description/common_platform.urdf.xacro'


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()


    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}] # add other parameters here if required
    )


    # Run the node
    ns = os.environ.get('ROS_NAMESPACE', '').strip()
    if ns:
        return LaunchDescription([
            GroupAction([
                PushRosNamespace(ns),
                node_robot_state_publisher
            ])
        ])
    else:
        return LaunchDescription([
            node_robot_state_publisher
        ])
