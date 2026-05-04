from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("abb_model"),
                "launch",
                "rsp_irb4600.launch.py"
            ])
        )
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen"
    )

    model_spawner = Node(
        package="final_project",
        executable="model_spawner",
        name="model_spawner",
        output="screen"
    )

    manipulator_executor = Node(
        package="final_project",
        executable="manipulator_executor",
        name="manipulator_executor",
        output="screen"
    )

    return LaunchDescription([
        robot_launch,
        rviz,
        model_spawner,
        manipulator_executor
    ])