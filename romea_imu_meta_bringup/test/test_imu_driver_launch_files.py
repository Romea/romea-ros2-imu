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
import yaml
from romea_imu_meta_bringup import IMUMetaDescription, generate_launch_file


def check_param(node, name, value):
    param = [d["value"] for d in node["param"] if d["name"] == name]
    assert param, f"param {name} is note defined in node configuration"
    assert (
        param[0] == value
    ), f"value of param {name} should be equal to {value} instead of {param[0]}"


def check_remap(node, from_, to_):
    remap = [d["to"] for d in node["remap"] if d["from"] == from_]
    assert remap, f"remap for {from_} is note defined in node configuration"
    assert remap[0] == to_, f"remap for {from_} should be equal to {to_} instead of {remap[0]}"


def get_meta_description(profile_filename):
    meta_description_file_path = os.path.join(os.getcwd(), profile_filename)
    return IMUMetaDescription(meta_description_file_path, "robot")


def get_nodes_configuration(profile_filename, mode="live"):
    meta_description = get_meta_description(profile_filename)
    launch_file = generate_launch_file(mode, meta_description)
    return yaml.safe_load(launch_file)["launch"][-1]["group"][2:]


def test_xsens_driver_launch_file():

    node = get_nodes_configuration("test_xsens_driver.yaml")[0]["node"]

    assert node["pkg"] == "xsens_driver"
    assert node["exec"] == "mtnode.py"
    assert node.get("namespace") is None
    assert "plugin" not in node
    check_param(node, "device", "/dev/ttyUSB0")
    check_param(node, "baudrate", 115200)
    check_remap(node, "imu/data", "data")
    check_remap(node, "imu/mag", "mag")
    check_remap(node, "imu/data_str", "data_str")


def test_bluespace_ai_xsens_mti_driver_launch_file():

    node = get_nodes_configuration("test_bluespace_ai_xsens_mti_driver.yaml")[0]["node"]

    assert node["pkg"] == "bluespace_ai_xsens_mti_driver"
    assert node["exec"] == "xsens_mti_node"
    assert node.get("namespace") is None
    assert "plugin" not in node
    # assert nodes_configuration["driver"]["parameters"]["frame_id"] == "robot_gps_link"
    check_param(node, "port", "/dev/ttyUSB0")
    check_param(node, "baudrate", 115200)
