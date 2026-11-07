import os.path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    log_level = DeclareLaunchArgument(
            "log_level",
            default_value=["info"],
            description="Logging level",
      )

    node = Node(
        package="eufs_sim2",
        executable="eufs_sim2_node",
        parameters=[
            {"core_params": get_package_share_directory("vehicle_models") +
             "/config/DynamicBicycle/ads-dv-calculated.yaml"},
            get_package_share_directory("eufs_sim2") + "/config/plugin_params.yaml"
        ],
        remappings=[
            ("/plugin/cone_fusion/gt_cones", "/cones/lenient"),
            ("/plugin/cone_fusion/cones", "/cones"),
            ("/plugin/cone_fusion/map", "/map"),
            ("/plugin/cone_fusion/ground_truth/map", "/ground_truth/map"),
            ("/plugin/wheel_speed_plugin/wheel_speed", "/ros_can/wheel_speeds"),
            ("/plugin/vehicle_state_plugin/ground_truth/state", "/odom"),
            ("/plugin/state_publisher/ros_can/state_str", "sim/ros_can/state_str"),
            ("/plugin/state_publisher/ros_can/state", "sim/ros_can/state"),
            ("/complete_mission_flag", "/ros_can/mission_completed"),
            ("/fix", "/ros_can/fix"),
            # For testing with cone_buffer
            ("/plugin/cone_fusion/camera/cones", "/camera/cones"),
            ("/plugin/cone_fusion/lidar_grid/cones", "/lidar_grid/cones"),
            # For testing sensor_fusion
            ("/plugin/twist_publisher/twist", "/ros_can/twist"),
            ("/plugin/imu_plugin/imu/data", "/imu/data"),
        ],
        output="screen",
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]

    )

    # Launch default map set by user

    map_lib = get_package_share_directory("map_lib")
    default_map = os.path.join(map_lib, "maps", "competitions", "FSUK", "2023", "trackdrive.csv")

    if not os.path.exists(default_map):
        raise FileNotFoundError(f"\033[91m{default_map} does not exist! \
                                Please check the file path again!\033[0m")

    service_call_cmd = [[
            FindExecutable(name='ros2'),
            " service call ",
            "/eufs_sim2/file ",
            "eufs_msgs/srv/SetString",
            " '{{data: \"{}\"}}'".format(default_map),
        ]]
    load_map = ExecuteProcess(
        cmd=service_call_cmd,
        shell=True
    )

    eufs_sim2_dir = get_package_share_directory("eufs_sim2")

    launch_static_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                eufs_sim2_dir,
                'launch/publish_urdf.launch.py'
                )
        ])
    )

    ld = LaunchDescription()
    ld.add_action(log_level)
    ld.add_action(launch_static_publisher)
    ld.add_action(node)
    ld.add_action(load_map)
    return ld
