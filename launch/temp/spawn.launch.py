import os
from launch import LaunchDescription
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def launch_setup(context, *args, **kwargs):

    x_spawn = LaunchConfiguration('x_spawn').perform(context)
    y_spawn = LaunchConfiguration('y_spawn').perform(context)
    Y_spawn = LaunchConfiguration('Y_spawn').perform(context)
    entity_name = LaunchConfiguration('entity_name').perform(context)

    # path & directory
    pkg_path = os.path.join(get_package_share_directory('nsbot'))
    robot_file_path = os.path.join(pkg_path,'description','robot.urdf.xacro')
    nav2_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup') 
    nav2_launch_dir = os.path.join(nav2_dir, 'launch') 
    static_map_path = os.path.join(pkg_path, 'maps', 'smalltown_world.yaml')
    nav2_params_path = os.path.join(pkg_path, 'params', 'nav2_params.yaml')
    nav2_bt_path = FindPackageShare(package='nav2_bt_navigator').find('nav2_bt_navigator')
    behavior_tree_xml_path = os.path.join(nav2_bt_path, 'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml')
    ekf_file='ekf_'+entity_name+'.yaml'
    robot_localization_file_path = os.path.join(pkg_path, 'config', ekf_file)
    
    
    
    # configuration variables    
    namespace = LaunchConfiguration('namespace')
    #namespace = entity_name
    use_namespace = LaunchConfiguration('use_namespace')
    #use_namespace = 'False'
    slam = LaunchConfiguration('slam')
    #slam = 'False'
    map_yaml_file = LaunchConfiguration('map')
    #map_yaml_file = static_map_path 
    use_sim_time = LaunchConfiguration('use_sim_time')
    #use_sim_time = 'True'
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    #default_bt_xml_filename = behavior_tree_xml_path
    autostart = LaunchConfiguration('autostart')  
    #autostart = 'True'  
    #params_file = LaunchConfiguration('params_file')
    params_file = nav2_params_path
   
    
    
   
  # map fully qualified names to relative ones so the node's namespace can be prepended.
  # in case of the transforms (tf), currently, there doesn't seem to be a better alternative
  # https://github.com/ros/geometry2/issues/32
  # https://github.com/ros/robot_state_publisher/pull/30
  # todo(orduno) substitute with `pushnoderemapping`
  #              https://github.com/ros2/launch_ros/issues/56
  #  remappings = [('/tf', 'tf'),
  #              ('/tf_static', 'tf_static')]


    # declare launch arguments
    x_spawn_arg = DeclareLaunchArgument('x_spawn', default_value='0.0')
    y_spawn_arg = DeclareLaunchArgument('y_spawn', default_value='0.0')
    Y_spawn_arg = DeclareLaunchArgument('Y_spawn', default_value='0.0')
    entity_name_arg = DeclareLaunchArgument('entity_name', default_value='robot_1')

    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='robot_1',
        description='top-level namespace')  
    declare_use_namespace_cmd = DeclareLaunchArgument(
        name='use_namespace',
        default_value='False',
        description='whether to apply a namespace to the navigation stack')    
    declare_slam_cmd = DeclareLaunchArgument(
        name='slam',
        default_value='False',
        description='whether to run slam') 
    declare_map_yaml_cmd = DeclareLaunchArgument(
        name='map',
        default_value=static_map_path,
        description='full path to map file to load')    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='use simulation (gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        name='params_file',
        default_value=nav2_params_path,
        description='full path to the ros2 parameters file to use for all launched nodes')
    declare_bt_xml_cmd = DeclareLaunchArgument(
        name='default_bt_xml_filename',
        default_value=behavior_tree_xml_path,
        description='full path to the behavior tree xml file to use')                
    declare_autostart_cmd = DeclareLaunchArgument(
        name='autostart', 
        default_value='True',
        description='automatically startup the nav2 stack')
    
   
    # start robot localization
    
    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name = 'ekf_filter_node',
        
        namespace='/'+entity_name,
        output='screen',
        parameters=[robot_localization_file_path, 
                {'frame_prefix': entity_name+'/','use_sim_time': use_sim_time}],
        #remappings=[('odometry/filtered',entity_name'/odometry/filtered'), 
                    #('set_pose',entity_name+'/set_pose')]
     )
   
    
    # create robot_state_publihser and joint_state_publisher nodes
    robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=entity_name,            
        parameters=[{'frame_prefix': entity_name+'/','use_sim_time': use_sim_time, 'robot_description': Command(['xacro ', robot_file_path, ' robot_name:=', entity_name])}],
        output="screen"
    )


    # spawn robot set gazebo
    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        namespace=entity_name,
        output='screen',
        arguments=['-entity', entity_name,
                   '-robot_namespace', entity_name,
                   '-x', x_spawn, '-y', y_spawn, '-Y', Y_spawn,
                   '-topic', 'robot_description',
                   '-timeout', '120.0'
                   ]
    )
    
    # launch nav2 stack
    nav_include = GroupAction(
        actions=[
            SetRemap(src='cmd_vel', dst='robot_1/cmd_vel'),
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
                launch_arguments = {'namespace': entity_name,
                        'use_namespace': use_namespace,
                        'slam': slam,
                        'map': static_map_path,
                        'use_sim_time': use_sim_time,
                        'params_file': params_file,
                        'default_bt_xml_filename': default_bt_xml_filename,
                        'autostart': autostart}.items()
             )
         ]
    )
     
    return [x_spawn_arg,y_spawn_arg,Y_spawn_arg, entity_name_arg,
            declare_namespace_cmd, declare_use_namespace_cmd,
            declare_slam_cmd, declare_map_yaml_cmd,
            declare_use_sim_time_cmd, declare_params_file_cmd,
            declare_bt_xml_cmd, declare_autostart_cmd,                     
            robot_state_publisher_node,     
            start_robot_localization_cmd,        
            start_gazebo_ros_spawner_cmd,
            #nav_include
            #start_ros2_navigation_cmd
            ]

def generate_launch_description():
    return LaunchDescription([
           OpaqueFunction(function = launch_setup)
           ])


