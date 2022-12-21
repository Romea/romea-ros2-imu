from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
)

from launch_ros.actions import PushRosNamespace
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from romea_common_bringup import device_link_name
from romea_imu_bringup import IMUMetaDescription

import yaml

def get_robot_namespace(context):
    return LaunchConfiguration("robot_namespace").perform(context)

def get_meta_description(context):
    
    meta_description_filename = LaunchConfiguration("meta_description_filename").perform(
        context
    )

    with open(meta_description_filename) as f:
        return IMUMetaDescription(yaml.safe_load(f))


def launch_setup(context, *args, **kwargs):

    robot_namespace = get_robot_namespace(context)
    meta_description = get_meta_description(context)

    if not meta_description.has_driver_configuration():
       return []

    imu_name = meta_description.get_name()

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("romea_imu_bringup"),
                        "launch",
                        "drivers/"+meta_description.get_driver_pkg() + ".launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "device": meta_description.get_device(),
            "baudrate": str(meta_description.get_baudrate()),
            "frame_id": device_link_name(robot_namespace,imu_name),
        }.items(),
    )

    return [
        GroupAction(
            actions=[
                PushRosNamespace(robot_namespace),
                PushRosNamespace(imu_name),
                driver,
            ]
        )
    ]


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument("meta_description_filename"))
    declared_arguments.append(DeclareLaunchArgument("robot_namespace",default_value=""))

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
