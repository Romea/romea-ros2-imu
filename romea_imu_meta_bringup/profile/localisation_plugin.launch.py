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

# from launch_ros.actions import SetRemap

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node, LoadComposableNodes
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode


class LaunchVariables():
    def __init__(self, context):
        self.__context = context

    def get(self, variable_name):
        return LaunchConfiguration(variable_name).perform(self.__context)


def launch_setup(context, *args, **kwargs):

    vars = LaunchVariables(context)

    mode = vars.get("mode")
    container = vars.get("container")
    restamping = vars.get("restamping")
    odom_topic = vars.get("odom_topic")

    # print(context.launch_configurations)

    imu_configuration = {
        "rate": float(vars.get("rate")),
        "acceleration_noise_density": float(vars.get("acceleration_noise_density")),
        "acceleration_bias_stability_std": float(vars.get("acceleration_bias_stability_std")),
        "acceleration_range": float(vars.get("acceleration_range")),
        "angular_speed_noise_density": float(vars.get("angular_speed_noise_density")),
        "angular_speed_bias_stability_std": float(vars.get("angular_speed_bias_stability_std")),
        "angular_speed_range": float(vars.get("angular_speed_range")),
        "magnetic_noise_density": float(vars.get("magnetic_noise_density")),
        "magnetic_bias_stability_std": float(vars.get("magnetic_bias_stability_std")),
        "magnetic_range": float(vars.get("magnetic_range")),
        "heading_std": float(vars.get("heading_std")),
        "xyz": [float(v) for v in vars.get("xyz")[1:-1].split(",")],
        "rpy": [float(v) for v in vars.get("rpy")[1:-1].split(",")],
    }

    common_arguments = {
        "package": "romea_localisation_imu_plugin",
        "name": "localisation_plugin",
        "parameters": [
            {
                "restamping": bool(restamping),
                "enable_accelerations": "live" in mode,
                "imu": imu_configuration
            }
        ],
        "remappings": [
            ("imu/data", "data"),
            ("vehicle_controller/odom", odom_topic)
        ]
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
                composable_node_descriptions=[
                    ComposableNode(**common_arguments, plugin=plugin)
                ],
            )
        )

    return [launch]


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument("odom_topic"),
            DeclareLaunchArgument("restamping", default_value="false"),
            DeclareLaunchArgument("container", default_value=""),
            OpaqueFunction(function=launch_setup)
        ]
    )
