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

from math import pi

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


class LaunchVariables:
    def __init__(self, context):
        self.__context = context

    def get(self, variable_name):
        return LaunchConfiguration(variable_name).perform(self.__context)


def launch_setup(context, *args, **kwargs):

    var = LaunchVariables(context)

    mode = var.get("mode")
    container = var.get("container")
    restamping = var.get("restamping")
    odom_topic = var.get("odom_topic")
    robot_namespace = var.get("robot_namespace")
    # print(context.launch_configurations)

    imu_configuration = {
        "rate": float(var.get("rate")),
        "acceleration_noise_density": float(var.get("acceleration_noise_density")),
        "acceleration_bias_stability_std": float(var.get("acceleration_bias_stability_std")),
        "acceleration_range": float(var.get("acceleration_range")),
        "angular_speed_noise_density": float(var.get("angular_speed_noise_density")) * pi / 180.0,
        "angular_speed_bias_stability_std": float(var.get("angular_speed_bias_stability_std"))
        * pi
        / 180.0
        / 3600.0,
        "angular_speed_range": float(var.get("angular_speed_range")),
        "magnetic_noise_density": float(var.get("magnetic_noise_density")),
        "magnetic_bias_stability_std": float(var.get("magnetic_bias_stability_std")),
        "magnetic_range": float(var.get("magnetic_range")),
        "heading_std": float(var.get("heading_std")) * pi / 180.0,
        "xyz": [float(v) for v in var.get("xyz")[1:-1].split(",")],
        "rpy": [float(v) for v in var.get("rpy")[1:-1].split(",")],
    }

    common_arguments = {
        "package": "romea_localisation_imu_plugin",
        "name": "localisation_plugin",
        "parameters": [
            {
                "restamping": restamping == "true",
                "enable_accelerations": "simulation" not in mode,
                "imu": imu_configuration,
                "use_sim_time": "live" not in mode,
                "debug": True,
            }
        ],
        "remappings": [
            ("imu/data", "data"),
            ("vehicle_controller/odom", odom_topic),
            ("attitude", f"/{robot_namespace}/localisation/attitude"),
            ("angular_speed", f"/{robot_namespace}/localisation/angular_speed"),
        ],
    }

    launch = LaunchDescription()
    if container == "":
        executable = "imu_localisation_plugin_node"
        launch.add_action(Node(**common_arguments, executable=executable))
    else:
        plugin = "romea::ros2::ImuLocalisationPlugin"
        launch.add_action(
            LoadComposableNodes(
                target_container=container,
                composable_node_descriptions=[ComposableNode(**common_arguments, plugin=plugin)],
            )
        )

    return [launch]


def generate_launch_description():

    default_odom_topic = [
        TextSubstitution(text="/"),
        LaunchConfiguration("robot_namespace"),
        TextSubstitution(text="/base/controller/odom"),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("odom_topic", default_value=default_odom_topic),
            DeclareLaunchArgument("restamping", default_value="false"),
            DeclareLaunchArgument("container", default_value=""),
            OpaqueFunction(function=launch_setup),
        ]
    )
