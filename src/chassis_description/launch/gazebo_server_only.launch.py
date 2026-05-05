import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_chassis = get_package_share_directory("chassis_description")

    urdf_path = os.path.join(pkg_chassis, "urdf", "gazebo_chassis_diff.urdf")

    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_description = f.read()

    # spawn_entity.py 不喜欢带 encoding 的 XML 声明，去掉第一行
    robot_description = (
        robot_description.replace('<?xml version="1.0" encoding="utf-8"?>', "")
        .replace('<?xml version="1.0" encoding="UTF-8"?>', "")
        .replace("<?xml version='1.0' encoding='utf-8'?>", "")
        .replace("<?xml version='1.0' encoding='UTF-8'?>", "")
        .strip()
    )

    # Gazebo 有时不能正确解析 package://，直接改成 file:// 绝对路径
    robot_description = robot_description.replace(
        "package://chassis_description/", "file://" + pkg_chassis + "/"
    )

    # 只启动 gzserver，不启动 gzclient
    # 这样可以避免 Gazebo 图形界面因为复杂 STL / collision 卡死
    gzserver = ExecuteProcess(
        cmd=[
            "gzserver",
            "--verbose",
            os.path.join(pkg_chassis, "worlds", "test_nav_world.world"),
            "/opt/ros/humble/lib/libgazebo_ros_init.so",
            "-s",
            "/opt/ros/humble/lib/libgazebo_ros_factory.so",
            "-s",
            "/opt/ros/humble/lib/libgazebo_ros_state.so",
        ],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "chassis",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.0",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            gzserver,
            robot_state_publisher,
            TimerAction(period=5.0, actions=[spawn_robot]),
        ]
    )
