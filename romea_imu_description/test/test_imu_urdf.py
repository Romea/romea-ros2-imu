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


import xml.etree.ElementTree as ET

from romea_imu_description import generate_urdf_description as urdf


def urdf_xml(mode):
    prefix = "robot_"
    name = "imu"

    description = {
        "manufacturer": "xsens",
        "model": "mti",
        "version": "",
        "rate": 100,
    }

    location = {"parent_link": "base_link", "xyz": [1.0, 2.0, 3.0], "rpy": [4.0, 5.0, 6.0]}

    ros_namespace = "ns"

    urdf_txt = urdf(prefix, mode, name, description, location, ros_namespace)

    with open("/tmp/test_imu_urdf_" + mode, "w") as file:
        file.write(urdf_txt)

    return ET.fromstring(urdf_txt)


def test_imu_name():
    assert urdf_xml("simulation").find("link").get("name") == "robot_imu_link"


def test_imu_position():
    assert urdf_xml("simulation").find("joint/origin").get("xyz") == "1.0 2.0 3.0"


def test_imu_orientation():
    assert (
        urdf_xml("simulation").find("joint/origin").get("rpy")
        == "0.06981317007977318 0.08726646259971647 0.10471975511965977"
    )


def test_imu_parent_link():
    assert urdf_xml("simulation").find("joint/parent").get("link") == "robot_base_link"


def test_imu_rate():
    assert urdf_xml("simulation").find("gazebo/sensor/update_rate").text == "100"


def test_angular_velocity_std_dev():
    assert (
        urdf_xml("simulation").find("gazebo/sensor/imu/angular_velocity/x/noise/stddev").text
        == "0.01370083462815549"
    )


def test_linear_accerleration_std_dev():
    assert (
        urdf_xml("simulation").find("gazebo/sensor/imu/linear_acceleration/x/noise/stddev").text
        == "0.00785"
    )


def test_plugin_gazebo_classic_ros_namespace():
    assert (
        urdf_xml("simulation_gazebo_classic").find("gazebo/sensor/plugin/ros/namespace").text
        == "ns"
    )


def test_plugin_gazebo_classic_filename():
    assert (
        urdf_xml("simulation_gazebo_classic").find("gazebo/sensor/plugin").get("filename")
        == "libgazebo_ros_imu_sensor.so"
    )


def test_plugin_gazebo_classic_frame_name():
    assert (
        urdf_xml("simulation_gazebo_classic").find("gazebo/sensor/plugin/frame_name").text
        == "robot_imu_link"
    )


def test_plugin_gazebo_filename():
    assert (
        urdf_xml("simulation").find("gazebo/sensor/plugin").get("filename") == "gz-sim-imu-system"
    )
