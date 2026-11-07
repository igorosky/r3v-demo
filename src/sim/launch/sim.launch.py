"""The whole simulation: Gazebo with room.sdf, the scout robot, the ROS bridge, the nodes moving
the cart, the head and the robot, and r3v_viewer.

Arguments:
- gui: Gazebo's GUI and RViz on the X display (1/true), default the GUI environment variable;
  without it Gazebo runs headless, rendering only for the lidar;
- patrol: drive the robot around the room (1/true), default the PATROL environment variable or
  true; false leaves /cmd_vel to teleop;
- viewer: run r3v_viewer, streaming to Janus (1/true), default the VIEWER environment variable or
  true;
- world: the SDF world file (default room.sdf of this package).
- rtp_url: the viewer's Janus RTP destination, default RTP_URL or localhost:5004.
- voxelizer_backend: single_threaded (default), multi_threaded or wgpu.
- voxelizer_threads: CPU worker threads, 0 (default) selects automatically.
- voxelizer_wgpu_bucket_count: GPU hashmap buckets (default 131072).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

SHARE = get_package_share_directory("r3v_sim")
URDF = os.path.join(SHARE, "urdf", "scout.urdf")
# The ground truth pose on /odom is the world pose, so this is where the robot starts in odom.
SPAWN = {"x": "0.0", "y": "-1.5", "z": "0.1"}


def is_true(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ("1", "true")


def launch_setup(context):
    gui = is_true(context, "gui")
    world = LaunchConfiguration("world").perform(context)
    rtp_url = LaunchConfiguration("rtp_url").perform(context)
    with open(URDF) as f:
        robot_description = f.read()
    sim_time = {"use_sim_time": True}

    gz_args = f"-r -v 2 {world}" if gui else f"-r -s --headless-rendering -v 2 {world}"
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
            ),
            launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items(),
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=["-world", "room", "-file", URDF, "-name", "scout"]
            + [arg for axis, value in SPAWN.items() for arg in (f"-{axis}", value)],
            output="screen",
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            parameters=[{"config_file": os.path.join(SHARE, "config", "bridge.yaml")}],
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}, sim_time],
            output="screen",
        ),
        # For web pages (roslib), e.g. r3v-web's janus example sending the renderer's camera topics.
        Node(
            package="rosbridge_server",
            executable="rosbridge_websocket",
            parameters=[{"port": 9090}],
        ),
        Node(
            package="r3v_sim",
            executable="oscillator.py",
            name="cart_oscillator",
            parameters=[{"topic": "/sim/cart/cmd_pos", "amplitude": 1.5, "period": 12.0}, sim_time],
        ),
        Node(
            package="r3v_sim",
            executable="oscillator.py",
            name="head_oscillator",
            parameters=[{"topic": "/turret/tilt_pos", "amplitude": 0.3, "period": 8.0}, sim_time],
        ),
    ]
    if is_true(context, "patrol"):
        actions.append(
            Node(package="r3v_sim", executable="patrol.py", parameters=[sim_time], output="screen")
        )
    if is_true(context, "viewer"):
        actions.append(
            Node(
                package="r3v_viewer",
                executable="r3v_viewer",
                parameters=[
                    {
                        "urdf": URDF,
                        "rtp_url": rtp_url,
                        "voxelizer.backend": LaunchConfiguration("voxelizer_backend"),
                        "voxelizer.threads": ParameterValue(
                            LaunchConfiguration("voxelizer_threads"), value_type=int
                        ),
                        "voxelizer.wgpu_bucket_count": ParameterValue(
                            LaunchConfiguration("voxelizer_wgpu_bucket_count"), value_type=int
                        ),
                    },
                    sim_time,
                ],
                output="screen",
            )
        )
    if gui:
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", os.path.join(SHARE, "config", "view.rviz")],
                parameters=[sim_time],
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value=EnvironmentVariable("GUI", default_value="0")),
            DeclareLaunchArgument("patrol", default_value=EnvironmentVariable("PATROL", default_value="1")),
            DeclareLaunchArgument("viewer", default_value=EnvironmentVariable("VIEWER", default_value="1")),
            DeclareLaunchArgument("world", default_value=os.path.join(SHARE, "worlds", "room.sdf")),
            DeclareLaunchArgument(
                "rtp_url",
                default_value=EnvironmentVariable(
                    "RTP_URL", default_value="rtp://127.0.0.1:5004?pkt_size=1200"
                ),
                description="Janus RTP destination for the viewer",
            ),
            DeclareLaunchArgument(
                "voxelizer_backend",
                default_value=EnvironmentVariable("VOXELIZER_BACKEND", default_value="single_threaded"),
                choices=["single_threaded", "multi_threaded", "wgpu"],
                description="Voxelizer implementation",
            ),
            DeclareLaunchArgument(
                "voxelizer_threads",
                default_value=EnvironmentVariable("VOXELIZER_THREADS", default_value="0"),
                description="CPU worker threads; 0 selects automatically",
            ),
            DeclareLaunchArgument(
                "voxelizer_wgpu_bucket_count",
                default_value=EnvironmentVariable("VOXELIZER_WGPU_BUCKET_COUNT", default_value="131072"),
                description="GPU voxel hashmap bucket count",
            ),
            # Gazebo finds package://r3v_sim/meshes/... as model://r3v_sim/meshes/... here.
            AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.dirname(SHARE)),
            OpaqueFunction(function=launch_setup),
        ]
    )
