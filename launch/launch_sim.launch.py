import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument


def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!
    
    package_name='my_bot' #<--- CHANGE ME

    pkg_share = FindPackageShare(package='my_bot').find('my_bot')
    world_file_name = 'obstacles.world'
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)
    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to the world model file to load')

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py')])
                , launch_arguments={'use_sim_time': 'true'}.items()
    )


    gazebo_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'gazebo_params.yaml')
    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-agrs --params-file'+gazebo_params_file}.items()
            )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')
    
  
    # twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    # twist_mux = Node(
    #         package="twist_mux",
    #         executable="twist_mux",
    #         parameters=[twist_mux_params],
    #         remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    #     )
    # delayed_rviz2 = TimerAction(period=10.0, actions=[rviz2])

    # Launch them all!
    return LaunchDescription([
        declare_world_cmd,
        rsp,
        gazebo,
        spawn_entity,
    ])
# Run the following:
# ros2 launch my_bot launch_sim.launch.py 
# ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
# ros2 launch my_bot online_async_launch.py use_sim_time:=true     
# ros2 launch nav2_bringup localization_launch.py map:=./my_map_save.yaml use_sim_time:=true 
# ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true map_subscribe_transient_local:=true
# ros2 run my_command my_commander
# ros2 run teleop_twist_keyboard teleop_twist_keyboard 
# --------------------------------------------------------

