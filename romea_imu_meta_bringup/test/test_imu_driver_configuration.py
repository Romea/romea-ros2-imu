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


import os
from romea_imu_meta_bringup import IMUMetaDescription, get_driver_launch_file_configuration


def get_meta_description(profile_filename):
    meta_description_file_path = os.path.join(os.getcwd(), profile_filename)
    return IMUMetaDescription(meta_description_file_path, "robot")


def get_nodes_configuration(profile_filename, mode="live"):
    meta_description = get_meta_description(profile_filename)
    return get_driver_launch_file_configuration(meta_description, mode)


def test_xsens_driver_configuration():

    nodes_configuration = get_nodes_configuration("test_xsens_driver_parameters.yaml")

    assert nodes_configuration["driver"]["package"] == "xsens_driver"
    assert nodes_configuration["driver"]["executable"] == "mtnode.py"
    assert nodes_configuration["driver"].get("namespace") is None
    assert "plugin" not in nodes_configuration["driver"]
    assert "component_container" not in nodes_configuration["driver"]
    # assert nodes_configuration["driver"]["parameters"]["frame_id"] == "robot_gps_link"
    assert nodes_configuration["driver"]["parameters"]["device"] == "/dev/ttyUSB0"
    assert nodes_configuration["driver"]["parameters"]["baudrate"] == 115200
    assert nodes_configuration["driver"]["remappings"]["imu/data"] == "data"
    assert nodes_configuration["driver"]["remappings"]["imu/mag"] == "mag"
    assert nodes_configuration["driver"]["remappings"]["imu/data_str"] == "data_str"


def test_bluespace_ai_xsens_mti_driver_configuration():

    nodes_configuration = get_nodes_configuration(
        "test_bluespace_ai_xsens_mti_driver_profile.yaml")

    assert nodes_configuration["driver"]["package"] == "bluespace_ai_xsens_mti_driver"
    assert nodes_configuration["driver"]["executable"] == "xsens_mti_node"
    assert nodes_configuration["driver"].get("namespace") is None
    assert "plugin" not in nodes_configuration["driver"]
    assert "component_container" not in nodes_configuration["driver"]
    # assert nodes_configuration["driver"]["parameters"]["frame_id"] == "robot_gps_link"
    assert nodes_configuration["driver"]["parameters"]["port"] == "/dev/ttyUSB0"
    assert nodes_configuration["driver"]["parameters"]["baudrate"] == 115200
