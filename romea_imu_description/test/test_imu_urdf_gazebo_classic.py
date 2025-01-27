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


import pytest
import xml.etree.ElementTree as ET
from romea_imu_description import urdf


@pytest.fixture(scope="module")
def urdf_xml():
    prefix = "robot_"
    mode = "simulation"
    name = "imu"

    description = {
        "manufacturer": "xsens",
        "model": "mti",
        "rate": 100,
    }

    location = {
        "parent_link": "base_link",
        "xyz": [1.0, 2.0, 3.0],
        "rpy": [4.0, 5.0, 6.0]
    }

    ros_namespace = "ns"

    with open('/tmp/urdf', 'w') as file:
        file.write(urdf(prefix, mode, name, description, location, ros_namespace))

    return ET.fromstring(urdf(prefix, mode, name, description, location, ros_namespace))


def test_imu_name(urdf_xml):
    assert urdf_xml.find("link").get("name") == "robot_imu_link"


def test_imu_position(urdf_xml):
    assert urdf_xml.find("joint/origin").get("xyz") == "1.0 2.0 3.0"


def test_imu_orientation(urdf_xml):
    print("orientation", urdf_xml.find("joint/origin").get("rpy"))
    assert (
        urdf_xml.find("joint/origin").get("rpy")
        == "0.06981317007977318 0.08726646259971647 0.10471975511965977"
    )


def test_imu_parent_link(urdf_xml):
    assert urdf_xml.find("joint/parent").get("link") == "robot_base_link"


def test_imu_rate(urdf_xml):
    assert urdf_xml.find("gazebo/sensor/update_rate").text == "100"


def test_plugin_namespace(urdf_xml):
    assert urdf_xml.find("gazebo/sensor/plugin/ros/namespace").text == "ns"


def test_plugin_frame_name(urdf_xml):
    assert urdf_xml.find("gazebo/sensor/plugin/frame_name").text == "robot_imu_link"


def test_angular_velocity_std_dev(urdf_xml):
    assert (
        urdf_xml.find("gazebo/sensor/imu/angular_velocity/x/noise/stddev").text
        == "0.01370083462815549"
    )


def test_linear_accerleration_std_dev(urdf_xml):
    assert (
        urdf_xml.find("gazebo/sensor/imu/linear_acceleration/x/noise/stddev").text
        == "0.00785"
    )
