import os
import signal
import subprocess
import time
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.conditions import IfCondition
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import Node


# Pattern dei processi lanciati da `gz sim` (il wrapper rinomina server e gui).
GZ_PROCESS_PATTERN = "^gz sim (server|gui)"


def kill_stale_gazebo(context=None, timeout=5.0):
    """Termina i server/gui Gazebo rimasti vivi da un lancio precedente.

    Con Ctrl+C il wrapper `gz sim` termina ma il server puo' restare orfano:
    al lancio successivo il nuovo server condivide il nome del mondo con quello
    vecchio e GUI e spawner parlano con lui, mostrando il robot dove era stato
    lasciato invece che nella posa iniziale.
    """
    subprocess.run(["pkill", "-TERM", "-f", GZ_PROCESS_PATTERN], check=False)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        alive = subprocess.run(
            ["pgrep", "-f", GZ_PROCESS_PATTERN],
            stdout=subprocess.DEVNULL,
            check=False,
        )
        if alive.returncode != 0:
            return []
        time.sleep(0.2)
    subprocess.run(
        ["pkill", f"-{signal.SIGKILL.value}", "-f", GZ_PROCESS_PATTERN],
        check=False,
    )
    return []


def install_sighup_cleanup(context):
    """Chiude Gazebo anche quando il terminale viene chiuso (SIGHUP).

    Con SIGHUP ros2 launch muore senza eseguire OnShutdown, e server/gui Gazebo,
    che `gz sim` mette in process group separati, non ricevono il segnale.
    """
    signal.signal(
        signal.SIGHUP,
        # Termina server/gui Gazebo, poi esce come farebbe il SIGHUP di default.
        lambda *_: (kill_stale_gazebo(timeout=3.0), os._exit(128 + signal.SIGHUP)),
    )
    return []


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

    rviz_nav2_config = os.path.join(
        gazebo_share,
        "config",
        "sweeper_nav2.rviz",
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
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    launch_rviz = LaunchConfiguration("launch_rviz")
    world = LaunchConfiguration("world")
    models_path = LaunchConfiguration("models_path")
    nav2 = LaunchConfiguration("nav2")
    map_yaml = LaunchConfiguration("map")
    nav2_params = LaunchConfiguration("nav2_params")

    world_path = PathJoinSubstitution([gazebo_share, "worlds", world])

    gz_resource_path = [
        models_path,
        os.pathsep,
        os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
    ]

    cleanup_stale_gazebo = OpaqueFunction(function=kill_stale_gazebo)

    cleanup_gazebo_on_shutdown = RegisterEventHandler(
        OnShutdown(on_shutdown=[OpaqueFunction(function=kill_stale_gazebo)]),
    )

    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-v", "3", world_path],
        additional_env={
            "GZ_SIM_SYSTEM_PLUGIN_PATH": gz_system_plugin_path,
            "GZ_SIM_RESOURCE_PATH": gz_resource_path,
            # Nasconde i warning interni di Qt della GUI di Gazebo ("Binding loop
            # detected" nei dialoghi QML); i warning di Gazebo restano visibili.
            "QT_LOGGING_RULES": "default.warning=false",
        },
        output="screen",
        # Se Gazebo termina (es. finestra chiusa) chiude tutto il launch.
        on_exit=Shutdown(reason="Gazebo terminato"),
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
            "-x", spawn_x,
            "-y", spawn_y,
            "-z", "0.067",
            "-Y", spawn_yaw,
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
            # Il remapping va al nodo del controller: passarlo al
            # controller_manager tramite il plugin Gazebo e' deprecato.
            "--controller-ros-args", "-r /tricycle_controller/cmd_vel:=/cmd_vel",
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

    # Senza Nav2 RViz resta su odom.
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{
            "use_sim_time": use_sim_time,
        }],
        condition=IfCondition(
            AndSubstitution(launch_rviz, NotSubstitution(nav2))
        ),
        output="screen",
    )

    # Con Nav2 attivo RViz usa map come frame fisso e mostra mappa e costmap.
    rviz_nav2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_nav2_config],
        parameters=[{
            "use_sim_time": use_sim_time,
        }],
        condition=IfCondition(AndSubstitution(launch_rviz, nav2)),
        output="screen",
    )

    # Nessuna localizzazione: /odom e' il ground truth di Gazebo, espresso nel
    # frame del mondo. Il frame della mappa coincide con quello del mondo,
    # quindi map -> odom e' l'identita'. Pubblicata sempre, anche senza Nav2,
    # per posizionare la mesh del mondo in RViz.
    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom",
        arguments=[
            "--frame-id", "map",
            "--child-frame-id", "odom",
        ],
        parameters=[{
            "use_sim_time": use_sim_time,
        }],
        output="screen",
    )

    # (pacchetto, eseguibile/nome del nodo)
    nav2_servers = [
        ("nav2_map_server", "map_server"),
        ("nav2_planner", "planner_server"),
        ("nav2_controller", "controller_server"),
        ("nav2_behaviors", "behavior_server"),
        ("nav2_bt_navigator", "bt_navigator"),
    ]
    nav2_nodes = [name for _, name in nav2_servers]

    # Stessa mesh del mondo Gazebo, visualizzata in RViz nel frame map.
    map_mesh_publisher = Node(
        package="autosweep_gazebo",
        executable="map_mesh_publisher.py",
        parameters=[{
            "use_sim_time": use_sim_time,
        }],
        output="screen",
    )

    nav2_actions = []
    for package, name in nav2_servers:
        parameters = [nav2_params, {"use_sim_time": use_sim_time}]
        if name == "map_server":
            parameters.append({"yaml_filename": map_yaml})
        nav2_actions.append(Node(
            package=package,
            executable=name,
            name=name,
            parameters=parameters,
            output="screen",
        ))

    nav2_actions.append(Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": nav2_nodes,
        }],
        output="screen",
    ))

    # Nav2 parte solo quando il robot e' spawnato e il tricycle_controller
    # accetta /cmd_vel.
    start_nav2 = RegisterEventHandler(
        OnProcessExit(
            target_action=tricycle_controller_spawner,
            on_exit=[GroupAction(nav2_actions, condition=IfCondition(nav2))],
        ),
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
        # In (0, 0) il footprint del robot si sovrappone a celle occupate della
        # mappa coverage1 e il planner rifiuta la posa di partenza.
        DeclareLaunchArgument(
            "spawn_x",
            default_value="0.5",
            description="Robot spawn x in the world/map frame [m]",
        ),
        DeclareLaunchArgument(
            "spawn_y",
            default_value="0.5",
            description="Robot spawn y in the world/map frame [m]",
        ),
        DeclareLaunchArgument(
            "spawn_yaw",
            default_value="0.0",
            description="Robot spawn yaw in the world/map frame [rad]",
        ),
        DeclareLaunchArgument(
            "nav2",
            default_value="true",
            choices=["true", "false"],
            description="Start the Nav2 go-to navigation",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=os.path.join(gazebo_share, "maps", "coverage1_map.yaml"),
            description="Occupancy grid yaml loaded by the Nav2 map server",
        ),
        DeclareLaunchArgument(
            "nav2_params",
            default_value=os.path.join(gazebo_share, "config", "nav2_params.yaml"),
            description="Nav2 parameters file",
        ),
        cleanup_stale_gazebo,
        cleanup_gazebo_on_shutdown,
        OpaqueFunction(function=install_sighup_cleanup),  # Pulizia anche alla chiusura del terminale.
        gazebo,
        bridge,
        state_publisher,
        odom_to_path,
        map_to_odom,
        map_mesh_publisher,
        start_joint_state_broadcaster,
        start_tricycle_controller,
        delayed_spawn,
        start_nav2,
        rviz,
        rviz_nav2,
    ])
