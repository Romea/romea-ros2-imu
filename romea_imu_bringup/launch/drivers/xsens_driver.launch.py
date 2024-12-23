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

import yaml


def launch_setup(context, *args, **kwargs):

    executable = LaunchConfiguration("executable").perform(context)
    executable_namespace = LaunchConfiguration("executable_namespace").perform(context)
    configuration_file_path = LaunchConfiguration("configuration_file_path").perform(context)

    driver = LaunchDescription()

    print(f'configuration_file_path: {configuration_file_path}')
    with open(configuration_file_path, 'r') as file:
        config_parameters = yaml.safe_load(file)

    driver_node = Node(
        package="xsens_driver",
        executable=executable,
        name="driver",
        namespace=executable_namespace,
        output="screen",
        parameters=[
            {"initial_wait": 1.0},
            {"timeout": 0.01},
            {"angular_velocity_covariance_diagonal": [0.0, 0.0, 0.0]},
            {"linear_acceleration_covariance_diagonal": [0.0, 0.0, 0.0]},
            {"orientation_covariance_diagonal": [0.0, 0.0, 0.0]},
            config_parameters,
        ],
        remappings=[
            ("imu/data", "data"),
            ("imu/mag", "mag"),
            ("imu/imu_data_str", "data_str"),
        ],
        arguments=[],
    )

    driver.add_action(driver_node)

    return [driver]


def generate_launch_description():

    declared_arguments = [
        DeclareLaunchArgument("executable"),
        DeclareLaunchArgument("executable_namespace", default_value=""),
        DeclareLaunchArgument("component_container", default_value=""),
        DeclareLaunchArgument("configuration_file_path"),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
