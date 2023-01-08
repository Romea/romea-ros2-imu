# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    port = LaunchConfiguration("device").perform(context)
    baudrate = LaunchConfiguration("baudrate").perform(context)
    frame_id = LaunchConfiguration("frame_id").perform(context)

    driver = LaunchDescription()

    print("bluespace")
    # Set env var to print messages to stdout immediately
    arg = SetEnvironmentVariable("RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED", "1")
    driver.add_action(arg)

    driver_node = Node(
        package="bluespace_ai_xsens_mti_driver",
        executable="xsens_mti_node",
        name="driver",
        output="screen",
        parameters=[
            {"scan_for_devices": False},
            {"port": port},
            {"baudrate": int(baudrate)},
            {"frame_id": frame_id},
            {"pub_imu": True},
            {"pub_quaternion": False},
            {"pub_mag": False},
            {"pub_angular_velocity": False},
            {"pub_acceleration": False},
            {"pub_free_acceleration": False},
            {"pub_dq": False},
            {"pub_dv": False},
            {"pub_sampletime": False},
            {"pub_temperature": False},
            {"pub_pressure": False},
            {"pub_gnss": False},
            {"pub_twist": False},
            {"pub_transform": False},
            {"pub_positionLLA": False},
            {"pub_velocity": False},
        ],
        arguments=[],
    )
    driver.add_action(driver_node)

    return [driver]


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument("device"))
    declared_arguments.append(DeclareLaunchArgument("baudrate"))
    declared_arguments.append(DeclareLaunchArgument("frame_id"))

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
