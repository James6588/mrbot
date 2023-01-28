import os 

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():



    pkg_share = FindPackageShare(package='mrbot').find('mrbot')    
    world_file_name = 'smalltown.world'
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)

    world = LaunchConfiguration('world')

    declare_world_cmd = DeclareLaunchArgument(
      name='world',
      default_value=world_path,
      description='Full path to the world model file to load')



    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')]),
      #condition=IfCondition(use_simulator),
      launch_arguments={'world': world}.items()
    )

    # Start Gazebo client    
    start_gazebo_client_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')]),
      #condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless]))
    )


    
    
    # Launch them all!
    return LaunchDescription([
        declare_world_cmd,
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,

    ])
