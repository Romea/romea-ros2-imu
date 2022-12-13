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

from romea_imu_bringup import (
    get_imu_name,
    has_imu_driver_configuration,
    get_imu_driver_pkg,
    get_imu_device,
    get_imu_baudrate,
)

import yaml

def get_robot_namespace(context):
    return LaunchConfiguration("robot_namespace").perform(context)

def get_meta_description(context):
    
    meta_description_filename = LaunchConfiguration("meta_description_filename").perform(
        context
    )

    with open(meta_description_filename) as f:
        return yaml.safe_load(f)


def launch_setup(context, *args, **kwargs):

    robot_namespace = get_robot_namespace(context)
    meta_description = get_meta_description(context)

    if not has_imu_driver_configuration(meta_description):
       return []

    imu_name = get_imu_name(meta_description)
    if robot_namespace != '':
        frame_id = robot_namespace + "_" + imu_name+"_link"
    else:
        frame_id = imu_name + "_link"

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("romea_imu_bringup"),
                        "launch",
                        "drivers/"+get_imu_driver_pkg(meta_description) + ".launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "device": get_imu_device(meta_description),
            "baudrate": str(get_imu_baudrate(meta_description)),
            "frame_id": frame_id,
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
