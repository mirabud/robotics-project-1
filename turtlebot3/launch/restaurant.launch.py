import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']   # waffle



def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world_name', default='turtlebot3_world')

    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3'), 'launch')

    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',value=[
        os.path.join("/opt/ros/humble", "share"),
        ":" +
        os.path.join(get_package_share_directory('turtlebot3'), "models")])
        

	# Spawn world with robot included
    world_entity = os.path.join(
        get_package_share_directory('turtlebot3'),
        "models", "worlds", "world_restaurant.sdf"
    )

    ignition_spawn_world = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
    )
    ignition_spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=['-entity', TURTLEBOT3_MODEL,
                   '-name', TURTLEBOT3_MODEL,
                   '-file', PathJoinSubstitution([
                        get_package_share_directory('turtlebot3'),
                        "models", "turtlebot3", TURTLEBOT3_MODEL+".sdf"]),
                   '-allow_renaming', 'true',
                   '-x', '-3.0',
                   '-y', '-3.0',
                   '-z', '10'],
        )
    world = os.path.join(get_package_share_directory('turtlebot3'), "models", "worlds", "world_restaurant.sdf")
    
    
    # Waiter Bot 
   
    
  
    return LaunchDescription([
        ign_resource_path,
        ignition_spawn_world,
        ignition_spawn_entity,
        
        # Include Gazebo and bridge launch files
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_ign_gazebo'),
                              'launch', 'ign_gazebo.launch.py')]),
            launch_arguments=[('ign_args', [' -r -v 3 ' + world])]),
                             
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/ros_ign_bridge.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/navigation2_restaurant.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

    ])
