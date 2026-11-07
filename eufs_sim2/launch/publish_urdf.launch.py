import os
import xacro
import json
from pathlib import Path
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_argument(context, arg):
    return LaunchConfiguration(arg).perform(context)


def save_urdf_path(urdf_path):
    EUFS_MASTER = Path(os.environ['EUFS_MASTER'])
    path = EUFS_MASTER/"data"/"current"
    # Make parent directories if they don't exist
    path.mkdir(parents=True, exist_ok=True)
    filePath = path/"setup.json"
    data = {}  # Empty JSON default value

    if os.path.exists(filePath):
        with open(filePath, 'r') as f:
            try:
                data = json.load(f)
            except json.JSONDecodeError:
                pass  # Empty JSON file
        os.remove(filePath)

    data["urdf_path"] = urdf_path
    with open(filePath, 'w') as f:
        json.dump(data, f, indent=4)


def load_urdf_and_transforms(context, *args, **kwargs):
    robot = get_argument(context, 'robot')
    save_urdf = get_argument(context, 'save_urdf') == 'True'

    xacro_path = os.path.join(get_package_share_directory('eufs_sim2'), 'eufs_racecar',
                              'robots', robot, 'robot.urdf.xacro')

    urdf_path = os.path.join(get_package_share_directory('eufs_sim2'), 'eufs_racecar',
                             'robots', robot, 'robot.urdf')

    if save_urdf:
        save_urdf_path(urdf_path)

    if not os.path.isfile(urdf_path):
        os.mknod(urdf_path)

    doc = xacro.process_file(
        xacro_path,
        )
    out = xacro.open_output(urdf_path)
    out.write(doc.toprettyxml(indent='  '))

    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {'robot_description': robot_description}
            ],
            remappings=[('/joint_states', '/eufs/joint_states')],
        )
    ]


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            default_value='eufs',
            description='Which robot to use in eufs_racecar',
            choices=os.listdir(
                os.path.join(
                    get_package_share_directory('eufs_sim2'),
                    'eufs_racecar', 'robots')
                    )
        ),
        DeclareLaunchArgument(
            'save_urdf',
            default_value='True',
            description='Save urdf_path as field in json file?',
            choices=['True', 'False']
        ),
        # Transforms
        OpaqueFunction(function=load_urdf_and_transforms),
        ])
