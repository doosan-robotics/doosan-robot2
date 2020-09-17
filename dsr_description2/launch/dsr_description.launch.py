import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from typing import List

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None


ARGUMENTS = [
    DeclareLaunchArgument(
        'model',
        default_value='m1013',
        description='Robot Model'
    )
    ]	

def generate_launch_description():
    
    #model11 = LaunchConfiguration('model').variable_name()
    print(1111111111111111111111111111111)
    print(LaunchConfiguration('model').variable_name())
    
    # Component yaml files are grouped in separate namespaces
    robot_description_config = load_file('dsr_description2', 'urdf/' + 'm1013' + '.urdf')
    robot_description = {'robot_description' : robot_description_config}

    # RViz
    rviz_config_file = get_package_share_directory('dsr_description2') + "/rviz/default.rviz"
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config_file],
                     parameters=[robot_description])

    # Static TF
    static_tf = Node(package='tf2_ros',
                     executable='static_transform_publisher',
                     name='static_transform_publisher',
                     output='log',
                     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base', 'base_0'])

    # Publish TF
    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 name='robot_state_publisher',
                                 output='both',
                                 parameters=[robot_description])

    joint_state_publisher_gui = Node(package='joint_state_publisher_gui',
                                    executable='joint_state_publisher_gui',
                                    name='joint_state_publisher_gui')

    return LaunchDescription(ARGUMENTS + [static_tf, robot_state_publisher, joint_state_publisher_gui, rviz_node])
