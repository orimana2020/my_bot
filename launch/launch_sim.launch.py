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
    
    diff_drive_spawner = Node(package="controller_manager", executable="spawner", arguments=["diff_cont"])
    joint_broad_spawner = Node(package="controller_manager", executable="spawner", arguments=["joint_broad"])

    # Rviz
    rviz_path='src/my_bot/config/my_bot.rviz'
    rviz2 = Node(package='rviz2', executable='rviz2', 
                 name="rviz2", output='screen',
                   arguments=['-d'+str(rviz_path), {'use_sim_time': 'true'}])
        
    # mappiung
    # online = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory(package_name),'launch','online_async.launch.py')])
    #             , launch_arguments={'use_sim_time': 'true'}.items() )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )
    delayed_rviz2 = TimerAction(period=5.0, actions=[rviz2])

    # Launch them all!
    return LaunchDescription([
        declare_world_cmd,
        rsp,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        delayed_rviz2,
        twist_mux,
    ])
# Run the following:

# ros2 launch my_bot launch_sim.launch.py 
# ros2 launch my_bot online_async_launch.py use_sim_time:=true
# ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
# ros2 run teleop_twist_keyboard teleop_twist_keyboard 
# --------------------------------------------------------

# no need to launch
# ros2 run rviz2 rviz2 -d src/my_bot/config/my_bot.rviz --ros-args -p use_sim_time:=true
# ros2 run twist_mux twist_mux --ros-args --params-file ./src/my_bot/config/twist_mux.yaml -r cmd_vel_out:=diff_cont/cmd_vel_unstamped
# ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
