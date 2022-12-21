import os
import pytest
import subprocess

from romea_imu_bringup import urdf_description

from ament_index_python import get_package_prefix

import xml.etree.ElementTree as ET


@pytest.fixture(scope="module")
def urdf():

    exe = (
        get_package_prefix("romea_imu_bringup")
        + "/lib/romea_imu_bringup/urdf_description.py"
    )

    meta_description_filename = os.path.join(os.getcwd(), "test_imu_bringup.yaml")
  
    return ET.fromstring(
        subprocess.check_output(
            [
                exe,
                "robot_namespace:robot",
                "meta_description_filename:" + meta_description_filename,
            ],
            encoding="utf-8",
        )
    )

def test_gps_name(urdf):
    assert urdf.find("link").get("name") == "robot_imu_link"


def test_gps_position(urdf):
    assert urdf.find("joint/origin").get("xyz") == "1.0 2.0 3.0"

def test_gps_position(urdf):
    assert urdf.find("joint/origin").get("rpy") == "4.0 5.0 6.0"


def test_gps_parent_link(urdf):
    assert urdf.find("joint/parent").get("link") == "robot_base_link"


def test_gps_rate(urdf):
    assert urdf.find("gazebo/sensor/update_rate").text == "100"
