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

from ament_index_python.packages import get_package_share_directory

from romea_imu_description import (
    get_imu_complete_configuration,
    get_imu_geometry_file_path,
    get_imu_geometry,
    get_imu_specifications_file_path,
    get_imu_specifications,
)


def test_get_imu_specifications_file_path_ok():
    assert (
        get_imu_specifications_file_path("xsens", "mti")
        == get_package_share_directory("romea_imu_description")
        + "/config/xsens_mti_specifications.yaml"
    )


def test_get_imu_specifications_ok():
    assert get_imu_specifications("xsens", "mti")['angular_speed_noise_density'] == 0.05


def test_get_imu_geometry_file_path_ok():
    assert (
        get_imu_geometry_file_path("xsens", "mti")
        == get_package_share_directory("romea_imu_description")
        + "/config/xsens_mti_geometry.yaml"
    )


def test_get_imu_geometry_ok():
    assert get_imu_geometry("xsens", "mti")['mass'] == 0.050


def test_get_gps_receiver_complete_configuration_ok():
    user_description = {
       "manufacturer": "xsens",
       "model": "mti",
       "rate": 100
    }

    configuration = get_imu_complete_configuration("imu", user_description)

    assert configuration["type"] == "AHRS"
    assert configuration["acceleration_noise_density"] == 0.0005
    assert configuration["angular_speed_noise_density"] == 0.05
    assert configuration["magnetic_noise_density"] == 5.e-08
    assert configuration["rate"] == 100
