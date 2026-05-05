import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
start_livox_like_filter = ExecuteProcess(
    cmd=[
        "python3",
        os.path.join(scripts_dir, "livox_like_filter.py"),
        "--ros-args",
        "-p",
        "use_sim_time:=true",
        "-p",
        "input_topic:=/livox/lidar",
        "-p",
        "output_topic:=/livox/lidar_like",
        "-p",
        "keep_ratio:=0.65",
        "-p",
        "max_points:=30000",
        "-p",
        "vertical_min_deg:=-7.0",
        "-p",
        "vertical_max_deg:=52.0",
    ],
    output="screen"
)
    pkg_chassis = get_package_share_directory("chassis_description")

    gazebo_server_launch = os.path.join(
        pkg_chassis,
        "launch",
        "gazebo_server_only.launch.py"
    )

    scripts_dir = os.path.expanduser(
        "~/chassis/src/chassis_description/scripts"
    )

    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_server_launch)
    )

    start_gzclient = ExecuteProcess(
        cmd=["gzclient"],
        output="screen"
    )

    start_rviz2 = ExecuteProcess(
        cmd=["rviz2"],
        output="screen"
    )

    start_joint_state_fix = ExecuteProcess(
        cmd=[
            "python3",
            os.path.join(scripts_dir, "gazebo_joint_state_fix.py"),
            "--ros-args",
            "-p",
            "use_sim_time:=true",
        ],
        output="screen"
    )

    start_odom_to_path = ExecuteProcess(
        cmd=[
            "python3",
            os.path.join(scripts_dir, "odom_to_path.py"),
            "--ros-args",
            "-p",
            "use_sim_time:=true",
        ],
        output="screen"
    )

    start_keyboard_drive = ExecuteProcess(
        cmd=[
            "python3",
            os.path.join(scripts_dir, "gazebo_keyboard_drive.py"),
        ],
        output="screen"
    )

    return LaunchDescription([
        start_gazebo_server,
        TimerAction(period=6.0, actions=[start_livox_like_filter]),
        TimerAction(period=3.0, actions=[start_gzclient]),
        TimerAction(period=4.0, actions=[start_rviz2]),
        TimerAction(period=5.0, actions=[start_joint_state_fix]),
        TimerAction(period=5.5, actions=[start_odom_to_path]),
        TimerAction(period=6.0, actions=[start_keyboard_drive]),
    ])
