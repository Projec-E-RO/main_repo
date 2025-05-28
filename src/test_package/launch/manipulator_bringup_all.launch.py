from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('open_manipulator_x_bringup'),
                'launch',
                'hardware.launch.py'
            ])
        ])
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('open_manipulator_x_moveit_config'),
                'launch',
                'moveit_core.launch.py'
            ])
        ])
    )

    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('open_manipulator_x_moveit_config'),
                'launch',
                'servo.launch.py'
            ])
        ])
    )

    gui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('open_manipulator_x_gui'),
                'launch',
                'open_manipulator_x_gui.launch.py'
            ])
        ])
    )

    return LaunchDescription([
        bringup_launch,
        moveit_launch,
        servo_launch,
        gui_launch
    ])

