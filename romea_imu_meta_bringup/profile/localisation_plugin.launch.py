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

from romea_common_meta_bringup import robot_namespace

class LaunchVariables():
    def __init__(self, context):
        self.__context = context

    def get(self, variable_name):
        return LaunchConfiguration(variable_name).perform(self.__context)


def launch_setup(context, *args, **kwargs):

    variables = LaunchVariables(context)

    mode = variables.get("mode")
    container = variables.get("container")
    restamping = variables.get("restamping")
    odom_topic = variables.get("odom_topic")

    # print(context.launch_configurations)

    rate = variables.get("rate")
    acceleration_noise_density = variables.get("acceleration_noise_density")
    acceleration_bias_stability_std = variables.get("acceleration_bias_stability_std")
    acceleration_range = variables.get("acceleration_range")
    angular_speed_noise_density = variables.get("angular_speed_noise_density")
    angular_speed_bias_stability_std = variables.get("angular_speed_bias_stability_std")
    angular_speed_range = variables.get("angular_speed_range")
    magnetic_noise_density = variables.get("magnetic_noise_density")
    magnetic_bias_stability_std = variables.get("magnetic_bias_stability_std")
    magnetic_range = variables.get("magnetic_range")
    heading_std = variables.get("heading_std")
    xyz = variables.get("xyz")[1:-1].split(",")
    rpy = variables.get("rpy")[1:-1].split(",")

    common_arguments = {
        "package": "romea_localisation_imu_plugin",
        "name": "localisation_plugin",
        "parameters": [
            {
                "restamping": bool(restamping),
                "enable_accelerations": "live" in mode,
                "imu":
                    {
                      "rate": float(rate),
                      "acceleration_noise_density": float(acceleration_noise_density),
                      "acceleration_bias_stability_std": float(acceleration_bias_stability_std),
                      "acceleration_range": float(acceleration_range),
                      "angular_speed_noise_density": float(angular_speed_noise_density),
                      "angular_speed_bias_stability_std": float(angular_speed_bias_stability_std),
                      "angular_speed_range": float(angular_speed_range),
                      "magnetic_noise_density": float(magnetic_noise_density),
                      "magnetic_bias_stability_std": float(magnetic_bias_stability_std),
                      "magnetic_range": float(magnetic_range),
                      "heading_std": float(heading_std),
                      "xyz":  [float(v) for v in xyz],
                      "rpy":  [float(v) for v in rpy],
                    },
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
