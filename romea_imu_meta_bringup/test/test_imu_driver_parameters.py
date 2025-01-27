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
from romea_imu_meta_bringup import IMUMetaDescription, get_complete_driver_parameters


def test_xsens_driver_parameters():

    meta_description_file_path = os.path.join(
        os.getcwd(), "test_xsens_driver_parameters.yaml"
    )
    meta_description = IMUMetaDescription(meta_description_file_path)

    parameters = get_complete_driver_parameters(meta_description, "robot")
    assert parameters["frame_id"] == "robot_imu_link"
    assert parameters["device"] == "/dev/ttyUSB0"
    assert parameters["baudrate"] == 115200


def test_bluespace_ai_xsens_mti_driver_parameters():

    meta_description_file_path = os.path.join(
        os.getcwd(), "test_bluespace_ai_xsens_mti_driver_parameters.yaml"
    )
    meta_description = IMUMetaDescription(meta_description_file_path)

    parameters = get_complete_driver_parameters(meta_description, "robot")
    assert parameters["frame_id"] == "robot_imu_link"
    assert parameters["port"] == "/dev/ttyUSB0"
    assert parameters["baudrate"] == 115200
