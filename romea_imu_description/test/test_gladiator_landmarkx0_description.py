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
    get_complete_configuration,
    get_geometry_file_path,
    get_geometry,
    get_specifications_file_path,
    get_specifications,
)

import pytest


@pytest.fixture(scope="module")
def user_description():

    return {
        "manufacturer": "gladiator",
        "model": "landmark",
        "version": "40",
        "rate": 100
    }


def test_get_specifications_file_path_ok(user_description):
    assert (
        get_specifications_file_path(user_description)
        == get_package_share_directory("romea_imu_description")
        + "/config/gladiator_landmark_x0_specifications.yaml"
    )


def test_get_specifications_ok(user_description):
    specifications = get_specifications(user_description)
    assert specifications["angular_speed_noise_density"]["dict"]["40"][300] == 0.004


def test_get_imu_geometry_file_path_ok(user_description):
    assert (
        get_geometry_file_path(user_description)
        == get_package_share_directory("romea_imu_description")
        + "/config/gladiator_landmark_x0_geometry.yaml"
    )


def test_get_geometry_ok(user_description):
    assert get_geometry(user_description)["mass"] == 0.113


def test_get_complete_configuration_ok(user_description):

    configuration = get_complete_configuration("imu", user_description, {})

    assert configuration["type"] == "AHRS"
    assert configuration["acceleration_noise_density"] == 1.1772e-03
    assert configuration["angular_speed_noise_density"] == 0.004
    assert configuration["magnetic_noise_density"] == 15.0e-9
    assert configuration["rate"] == 100
