import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_share = get_package_share_directory("autosweep_gazebo")
    #description_share = get_package_share_directory("autosweep_description")

    bridge_config = os.path.join(
        gazebo_share,
        "config",
        "sweeper_bridge_config.yaml",
    )

    controller_config = os.path.join(
        gazebo_share,
        "config",
        "sweeper_controllers.yaml",
    )

    rviz_config = os.path.join(
        gazebo_share,
        "config",
        "sweeper.rviz",
    )

    xacro_path = os.path.join(
        gazebo_share,
        "urdf",
        "sweeper_sim.urdf.xacro",
    )

    robot_description = xacro.process_file(xacro_path).toxml()

    gz_system_plugin_path = os.pathsep.join([
        os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
        os.environ.get("LD_LIBRARY_PATH", ""),
    ])

    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    world = LaunchConfiguration("world")
    models_path = LaunchConfiguration("models_path")

    world_path = PathJoinSubstitution([gazebo_share, "worlds", world])

    gz_resource_path = [
        models_path,
        os.pathsep,
        os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
    ]

    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-v", "3", world_path],
        additional_env={
            "GZ_SIM_SYSTEM_PLUGIN_PATH": gz_system_plugin_path,
            "GZ_SIM_RESOURCE_PATH": gz_resource_path,
        },
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{
            "config_file": bridge_config,
            "use_sim_time": use_sim_time,
        }],
        output="screen",
    )

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time,
        }],
        output="screen",
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world", "sweeper_world",
            "-name", "sweeper",
            "-topic", "/robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.067",
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--param-file", controller_config,
        ],
        output="screen",
    )

    tricycle_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "tricycle_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", controller_config,
        ],
        output="screen",
    )

    delayed_spawn = TimerAction(
        period=2.0,
        actions=[spawn_robot],
    )

    start_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    start_tricycle_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[tricycle_controller_spawner],
        )
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{
            "use_sim_time": use_sim_time,
        }],
        condition=IfCondition(launch_rviz),
        output="screen",
    )

    odom_to_path = Node(
        package="autosweep_gazebo",
        executable="odom_to_path.py",
        parameters=[{
            "use_sim_time": use_sim_time,
            "min_distance": 0.05,
            "min_angle": 0.05,
            "max_poses": 5000,
        }],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use the Gazebo simulation clock",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            choices=["true", "false"],
            description="Start RViz together with the simulation",
        ),
        DeclareLaunchArgument(
            "world",
            default_value="coverage1_world.sdf",
            description="World file inside autosweep_gazebo/worlds",
        ),
        DeclareLaunchArgument(
            "models_path",
            default_value=os.path.join(gazebo_share, "models"),
            description="Directory containing the map models (added to GZ_SIM_RESOURCE_PATH)",
        ),
        gazebo,
        bridge,
        state_publisher,
        odom_to_path,
        start_joint_state_broadcaster,
        start_tricycle_controller,
        delayed_spawn,
        rviz,
    ])
