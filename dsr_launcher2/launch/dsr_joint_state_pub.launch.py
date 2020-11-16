import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch.launch_context import LaunchContext

args =[ 
    DeclareLaunchArgument('name',  default_value = 'dsr01',     description = 'NAME_SPACE'     ),
    DeclareLaunchArgument('host',  default_value = '127.0.0.1', description = 'ROBOT_IP'       ),
    DeclareLaunchArgument('port',  default_value = '12345',     description = 'ROBOT_PORT'     ),
    DeclareLaunchArgument('mode',  default_value = 'virtual',   description = 'OPERATION MODE' ),
    DeclareLaunchArgument('model', default_value = 'm1013',     description = 'ROBOT_MODEL'    ),
    DeclareLaunchArgument('color', default_value = 'white',     description = 'ROBOT_COLOR'    ),
]
context = LaunchContext()

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


def generate_launch_description():

    #robot_description_config = load_file('dsr_description2', 'urdf/' + 'm1013' + '.urdf')
    #robot_description = {'robot_description' : robot_description_config}
    #print(get_package_share_directory('dsr_description2'))

    urdf = os.path.join( get_package_share_directory('dsr_description2'), 'urdf')
    xacro_path = os.path.join( get_package_share_directory('dsr_description2'), 'xacro')

    # RViz
    rviz_config_file = get_package_share_directory('dsr_description2') + "/rviz/default.rviz"
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config_file])

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
                                 parameters=[{
                                    'robot_description': Command(['xacro', ' ', xacro_path, '/', LaunchConfiguration('model'), '.urdf.xacro color:=', LaunchConfiguration('color')])           
                                 }])

    joint_state_publisher_gui = Node(package='joint_state_publisher_gui',
                                    executable='joint_state_publisher_gui',
                                    name='joint_state_publisher_gui')

    return LaunchDescription(args + [static_tf, robot_state_publisher, joint_state_publisher_gui, rviz_node])