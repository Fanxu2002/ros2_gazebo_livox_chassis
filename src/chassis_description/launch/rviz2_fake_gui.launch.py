import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_chassis = get_package_share_directory("chassis_description")

    urdf_path = os.path.join(
        pkg_chassis,
        "urdf",
        "rviz2_chassis.SLDASM.urdf"
    )

    scripts_dir = os.path.expanduser(
        "~/chassis/src/chassis_description/scripts"
    )

    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_description = f.read()

    robot_description = (
        robot_description.replace('<?xml version="1.0" encoding="utf-8"?>', "")
        .replace('<?xml version="1.0" encoding="UTF-8"?>', "")
        .replace("<?xml version='1.0' encoding='utf-8'?>", "")
        .replace("<?xml version='1.0' encoding='UTF-8'?>", "")
        .strip()
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": False,
            }
        ],
    )

    start_rviz2 = ExecuteProcess(
        cmd=["rviz2"],
        output="screen"
    )

    start_rviz2_gui_drive = ExecuteProcess(
        cmd=[
            "python3",
            os.path.join(scripts_dir, "rviz2_gui_drive.py"),
        ],
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher,
        TimerAction(period=1.0, actions=[start_rviz2]),
        TimerAction(period=2.0, actions=[start_rviz2_gui_drive]),
    ])
