# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    device = LaunchConfiguration("device").perform(context)
    baudrate = LaunchConfiguration("baudrate").perform(context)
    frame_id = LaunchConfiguration("frame_id").perform(context)
    mode = LaunchConfiguration("mode").perform(context)

    launch = LaunchDescription()

    if mode == "live":
        launch.add_action(
            Node(
                package="bluespace_ai_xsens_mti_driver",
                executable="xsens_mti_node",
                name="driver",
                output="screen",
                parameters=[
                    {
                        "port": device,
                        "baudrate": int(baudrate),
                        "scan_for_devices": False,
                        "frame_id": frame_id,
                        "pub_imu": True,
                        "pub_quaternion": False,
                        "pub_mag": False,
                        "pub_angular_velocity": False,
                        "pub_acceleration": False,
                        "pub_free_acceleration": False,
                        "pub_dq": False,
                        "pub_dv": False,
                        "pub_sampletime": False,
                        "pub_temperature": False,
                        "pub_pressure": False,
                        "pub_gnss": False,
                        "pub_twist": False,
                        "pub_transform": False,
                        "pub_positionLLA": False,
                        "pub_velocity": False
                    },
                ],
            )
        )

    return [launch]


def generate_launch_description():

    declared_arguments = [
        DeclareLaunchArgument("device"),
        DeclareLaunchArgument("baudrate")
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
