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
import pytest
from romea_imu_meta_bringup import (
    IMUMetaDescription,
    get_sensor_specifications,
    get_sensor_geometry,
    get_complete_sensor_configuration
)


@pytest.fixture(scope="module")
def meta_description():
    meta_description_file_path = os.path.join(os.getcwd(), "test_imu_meta_bringup.yaml")
    return IMUMetaDescription(meta_description_file_path)


def test_get_name(meta_description):
    assert meta_description.get_name() == "imu"


def test_get_namespace(meta_description):
    assert meta_description.get_namespace() == "ns"


def test_get_launch_file_configuration(meta_description):
    assert "imu_driver" in meta_description.get_launch_file_configuration()


def test_get_manufacturer(meta_description):
    assert meta_description.get_manufacturer() == "xsens"


def test_get_model(meta_description):
    assert meta_description.get_model() == "mti"


def test_get_rate(meta_description):
    assert meta_description.get_rate() == 100


def test_get_parent_link(meta_description):
    assert meta_description.get_parent_link() == "base_link"


def test_get_xyz(meta_description):
    assert meta_description.get_xyz() == [1.0, 2.0, 3.0]


def test_get_rpy(meta_description):
    assert meta_description.get_rpy() == [4.0, 5.0, 6.0]


def test_get_records(meta_description):
    records = meta_description.get_records()
    assert records["data"] is True


def test_get_sensor_specifications(meta_description):
    imu_specifactions = get_sensor_specifications(meta_description)
    assert imu_specifactions['acceleration_noise_density'] == 0.0005


def test_get_sensor_geometry(meta_description):
    imu_geometry = get_sensor_geometry(meta_description)
    assert imu_geometry['mass'] == 0.050


def test_get_complete_sensor_configuration(meta_description):
    imu_configuration = get_complete_sensor_configuration(meta_description)
    assert imu_configuration['acceleration_noise_density'] == 0.0005
