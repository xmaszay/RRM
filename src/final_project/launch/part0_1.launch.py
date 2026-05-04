from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    # Robot (URDF + TF)
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("abb_model"),
                "launch",
                "rsp_irb4600.launch.py"
            ])
        )
    )

    # RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen"
    )

    # Časť 0 – objekt
    model_spawner = Node(
        package="final_project",
        executable="model_spawner",
        name="model_spawner",
        output="screen"
    )

    # Časť 1 – pose teacher
    pose_teacher = Node(
        package="final_project",
        executable="pose_teacher",
        name="pose_teacher",
        output="screen"
    )

    return LaunchDescription([
        robot_launch,
        rviz,
        model_spawner,
        pose_teacher
    ])