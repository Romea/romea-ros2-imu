# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license

import os
import pytest
from numpy import radians
from romea_imu_bringup import IMUMetaDescription


@pytest.fixture(scope="module")
def meta_description():
    meta_description_filename = os.path.join(os.getcwd(), "test_imu_bringup.yaml")
    return IMUMetaDescription(meta_description_filename)


def test_get_name(meta_description):
    assert meta_description.get_name() == "imu"


def test_get_namespace(meta_description):
    assert meta_description.get_namespace() == "ns"


def test_has_driver_configuration(meta_description):
    assert meta_description.has_driver_configuration() is True


def test_get_driver_pkg(meta_description):
    assert meta_description.get_driver_pkg() == "xsens_driver"


def test_get_driver_device(meta_description):
    assert meta_description.get_driver_device() == "/dev/ttyUSB0"


def test_get_driver_baudrate(meta_description):
    assert meta_description.get_driver_baudrate() == 115200


def test_get_type(meta_description):
    assert meta_description.get_type() == "xsens"


def test_get_model(meta_description):
    assert meta_description.get_model() == "mti"


def test_get_rate(meta_description):
    assert meta_description.get_rate() == 100


def test_get_parent_link(meta_description):
    assert meta_description.get_parent_link() == "base_link"


def test_get_xyz(meta_description):
    assert meta_description.get_xyz() == [1.0, 2.0, 3.0]


def test_get_rpy_deg(meta_description):
    assert meta_description.get_rpy_deg() == [4.0, 5.0, 6.0]


def test_get_rpy_rad(meta_description):
    assert meta_description.get_rpy_rad() == radians([4.0, 5.0, 6.0]).tolist()
