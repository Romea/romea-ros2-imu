from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable,
)

from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context, *args, **kwargs):

    port = LaunchConfiguration("device").perform(context)
    baudrate = LaunchConfiguration("baudrate").perform(context)
    frame_id = LaunchConfiguration("frame_id").perform(context)

    driver = LaunchDescription()

    driver_node = Node(
            package='xsens_driver',
            executable='mtnode.py',
            name='driver',
            output='screen',
            parameters=[
                {"port": port},
                {"baudrate": int(baudrate)},
                {"frame_id": frame_id},
                {"initial_wait": 1.0},
                {"timeout": 0.01},
                {"angular_velocity_covariance_diagonal": [0.0, 0.0, 0.0]},
                {"linear_acceleration_covariance_diagonal": [0.0, 0.0, 0.0]},
                {"orientation_covariance_diagonal": [0.0, 0.0, 0.0]},
            ],
            remappings=[
                ("imu/data", "data"),
                ("imu/mag", "mag"),
                ("imu/imu_data_str", "data_str"),
            ],
            arguments=[]
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
