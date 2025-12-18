import os

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    OrSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # -------------------------
    # Launch arguments
    # -------------------------
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="iiwa_description",
            description=(
                'Package con la configurazione dei controller in "config".'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="iiwa_controllers.yaml",
            description="YAML con la configurazione dei controller iiwa.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="iiwa_description",
            description="Package con gli xacro/URDF di iiwa.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="iiwa.config.xacro",
            description="File URDF/XACRO del robot iiwa.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='"iiwa_"',
            description=(
                "Prefisso dei joint per setup multi-robot. "
                'Formato atteso: "<prefix>/".'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="iiwa",
            description=(
                "Namespace dei nodi iiwa. "
                'Formato atteso: "<ns>".'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim",
            default_value="true",
            description="Avvia il robot in simulazione Gazebo.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Usa fake hardware per iiwa.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_planning",
            default_value="false",
            description="Avvia MoveIt2 planning per iiwa.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_servoing",
            default_value="false",
            description="Avvia MoveIt2 servoing per iiwa.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "iiwa_controller",
            default_value="iiwa_arm_controller",
            description="Nome del controller principale di iiwa.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Start RViz2 automatically with this launch file.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.170.10.2",
            description="IP FRI di iiwa.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_port",
            default_value="30200",
            description="Port FRI di iiwa.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value="initial_positions.yaml",
            description="Config posizioni iniziali iiwa in simulazione.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "command_interface",
            default_value="position",
            description="Interfaccia di comando iiwa [position|velocity|effort].",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "base_frame_file",
            default_value="base_frame.yaml",
            description="Config frame base iiwa rispetto al world.",
        )
    )

    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    use_sim = LaunchConfiguration("use_sim")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_planning = LaunchConfiguration("use_planning")
    use_servoing = LaunchConfiguration("use_servoing")
    iiwa_controller = LaunchConfiguration("iiwa_controller")
    start_rviz = LaunchConfiguration('start_rviz')
    robot_ip = LaunchConfiguration("robot_ip")
    robot_port = LaunchConfiguration("robot_port")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    command_interface = LaunchConfiguration("command_interface")
    base_frame_file = LaunchConfiguration("base_frame_file")
    namespace = LaunchConfiguration("namespace")

    # -------------------------
    # iiwa: robot_description (via xacro)
    # -------------------------
    iiwa_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "config", description_file]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_sim:=",
            use_sim,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "robot_port:=",
            robot_port,
            " ",
            "initial_positions_file:=",
            initial_positions_file,
            " ",
            "command_interface:=",
            command_interface,
            " ",
            "base_frame_file:=",
            base_frame_file,
            " ",
            "description_package:=",
            description_package,
            " ",
            "runtime_config_package:=",
            runtime_config_package,
            " ",
            "controllers_file:=",
            controllers_file,
            " ",
            "namespace:=",
            namespace,
            
        ]
    )

    iiwa_description_param = {
        "robot_description": ParameterValue(iiwa_description_content, value_type=str)
    }

    # -------------------------
    # iiwa: MoveIt (opzionale)
    # -------------------------
    iiwa_planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("iiwa_bringup"),
                "/launch",
                "/iiwa_planning.launch.py",
            ]
        ),
        launch_arguments={
            "description_package": description_package,
            "description_file": description_file,
            "prefix": prefix,
            'start_rviz': start_rviz,
            "base_frame_file": base_frame_file,
            "namespace": namespace,
            "use_sim": use_sim,
        }.items(),
        condition=IfCondition(use_planning),
    )

    iiwa_servoing_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("iiwa_bringup"),
                "/launch",
                "/iiwa_servoing.launch.py",
            ]
        ),
        launch_arguments={
            "description_package": description_package,
            "description_file": description_file,
            "prefix": prefix,
            "base_frame_file": base_frame_file,
            "namespace": namespace,
        }.items(),
        condition=IfCondition(use_servoing),
    )

    iiwa_controllers_path = PathJoinSubstitution(
        [
            FindPackageShare(runtime_config_package),
            "config",
            controllers_file,
        ]
    )

    # -------------------------
    # fra2mo: robot_description da xacro
    # -------------------------
    fra2mo_xacro = os.path.join(
        get_package_share_directory("ros2_fra2mo"), "urdf", "fra2mo.urdf.xacro"
    )
    fra2mo_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", fra2mo_xacro]), value_type=str
        )
    }

    models_path = os.path.join(
        get_package_share_directory("ros2_fra2mo"), "models"
    )
    world_file = os.path.join(
        get_package_share_directory("ros2_fra2mo"), "worlds", "fra2mo_box_arena.sdf"
    )

    cm_manager_name = PathJoinSubstitution(["/", namespace, "controller_manager"])

    declared_arguments.append(
        DeclareLaunchArgument(
            "gz_args",
            default_value=[world_file, " -r"],
            description="Argomenti da passare a gz sim (world + opzioni).",
        )
    )

    gz_args = LaunchConfiguration("gz_args")

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[iiwa_description_param, iiwa_controllers_path],
        namespace=namespace,
        name="controller_manager",
        output="both",
        condition=UnlessCondition(use_sim),
    )

    iiwa_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        name="robot_state_publisher",  
        output="both",
        parameters=[iiwa_description_param, {"use_sim_time": True}],
    )

    fra2mo_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="fra2mo",
        output="both",
        parameters=[fra2mo_description, {"use_sim_time": True}],
    )

    fra2mo_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace="fra2mo", 
        name='joint_state_publisher',
        parameters=[{"use_sim_time": True}],
    )

    position_iiwa = [-5.15, 2.64, 0, 1.8]
    position_fra2mo = [5.85, -5.85, 0.1]

    iiwa_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", "iiwa",
            "-topic", "iiwa/robot_description",
            "-allow_renaming",
            "true",
            "-x",
            str(position_iiwa[0]),
            "-y",
            str(position_iiwa[1]),
            "-z",
            str(position_iiwa[2]),
            "-Y", 
            str(position_iiwa[3]),
        ],
        condition=IfCondition(use_sim),
    )

    fra2mo_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", "fra2mo",
            "-topic", "/fra2mo/robot_description",
            "-x",
            str(position_fra2mo[0]),
            "-y",
            str(position_fra2mo[1]),
            "-z",
            str(position_fra2mo[2]),
        ],
        condition=IfCondition(use_sim),
    )

    pose_saver_node = Node(
        package='ros2_fra2mo',
        executable='save_iiwa_pose.py',
        name='iiwa_pose_saver',
        output='screen'
    )

    aruco_node_fra2mo = Node(
        package='aruco_ros',
        executable='single',
        namespace='fra2mo',       
        name='aruco_single',
        parameters=[{
            'image_is_rectified': True,
            'marker_size': 0.1,          
            'marker_id': 50,            
            'reference_frame': '',       
            'camera_frame': 'fra2mo_d435_link', 
            'marker_frame': 'detected_marker'
        }],
        remappings=[
            ('/camera_info', '/fra2mo/camera/camera_info'),
            ('/image', '/fra2mo/camera/image'),
        ],
        output='screen'
    )

    aruco_node_iiwa = Node(
        package='aruco_ros',
        executable='single',
        namespace='iiwa',         
        name='aruco_single',
        parameters=[{
            'image_is_rectified': True,
            'marker_size': 0.1,          
            'marker_id': 50,                      
            'reference_frame': '',       
            'camera_frame': 'camera_link_optical', 
            'marker_frame': 'detected_marker'
        }],
        remappings=[
            ('/camera_info', '/iiwa/camera/camera_info'),
            ('/image', '/iiwa/camera/image')
        ],
        output='screen'
    )


    # -------------------------
    # Spawner controller iiwa
    # -------------------------
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,     
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            cm_manager_name, 
        ],
    )


    external_torque_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=[
            "ets_state_broadcaster",
            "--controller-manager",
            cm_manager_name,
        ],
        condition=UnlessCondition(use_sim),
    )


    iiwa_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=[
            iiwa_controller,
            "--controller-manager",
            cm_manager_name,
        ],
    )




    # -------------------------
    # Event handlers (ordini di startup)
    # -------------------------
    delay_jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=iiwa_spawn,
            on_exit=[joint_state_broadcaster_spawner],
        ),
        condition=IfCondition(use_sim),
    )

    delay_jsb_after_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[joint_state_broadcaster_spawner],
        ),
        condition=UnlessCondition(use_sim),
    )

    delay_iiwa_controller_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[iiwa_controller_spawner],
        )
    )

    delay_fra2mo_spawn_after_state_pub = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=fra2mo_state_publisher_node,
            on_start=[fra2mo_spawn],
        )
    )


    # -------------------------
    # Gazebo + bridge
    # -------------------------
    gazebo_ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
                )
            ]
        ),
        launch_arguments={"gz_args": gz_args}.items(),
        condition=IfCondition(use_sim),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="parameter_bridge",
        arguments=[
            # Clock Globale
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            
            # IIWA Topics
            "/iiwa/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/iiwa/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            
            # Odometria e TF 
            "/model/fra2mo/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
            "/model/fra2mo/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V",

            # FRA2MO - Topics
            "/fra2mo/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/fra2mo/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "/fra2mo/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
            "/fra2mo/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
        ],
        output="screen",
        condition=IfCondition(use_sim),
    )

    odom_tf = Node(
        package="ros2_fra2mo",
        executable="dynamic_tf_publisher",
        name="odom_tf",
        parameters=[{"use_sim_time": True}],
    )


    # -------------------------
    # Lista nodi da lanciare
    # -------------------------
    nodes_to_start = [

        # Gazebo + bridge
        gazebo_ignition,
        bridge,

        # Fra2mo Nodes
        fra2mo_state_publisher_node,
        fra2mo_joint_state_publisher,
        odom_tf,
        delay_fra2mo_spawn_after_state_pub,

        # Iiwa Nodes
        iiwa_state_pub_node,
        control_node,
        iiwa_spawn,
        delay_jsb_after_spawn,
        delay_jsb_after_control_node,
        delay_iiwa_controller_after_jsb,
        external_torque_broadcaster_spawner,

        # Viz & Tools
        iiwa_planning_launch,
        iiwa_servoing_launch,
        aruco_node_fra2mo,
        aruco_node_iiwa,
        pose_saver_node,

    ]

    # -------------------------
    # Env per le risorse Gazebo
    # -------------------------
    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=models_path + ":" + os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
    )

    return LaunchDescription(
        [set_gz_resource_path] + declared_arguments + nodes_to_start
    )
