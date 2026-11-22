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


import math
import os
import time
import unittest
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
import rclpy
from rclpy.qos import qos_profile_sensor_data
import romea_imu_meta_bringup.meta_description as meta_description
from sensor_msgs.msg import Imu


def generate_test_description():
    try:
        get_package_share_directory("gazebo_ros")
        simulator_type = "gazebo_classic"
    except PackageNotFoundError:
        simulator_type = "gazebo"

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("romea_imu_meta_bringup")
            + "/launch/simulation_test.launch.py"
        ),
        launch_arguments={
            "gui": "false",
            "simulator_type": simulator_type,
            "robot_namespace": "robot",
            "meta_description_file_path": os.path.join(
                os.path.dirname(__file__), "test_simulation.yaml"
            ),
        }.items(),
    )
    return LaunchDescription([
        simulation,
        launch_testing.actions.ReadyToTest(),
    ])


class TestIMUGazeboSimulation(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_imu_gazebo_simulation")
        cls.messages = []
        cls.subscription = cls.node.create_subscription(
            Imu, "/robot/ns/imu/data", cls.messages.append, qos_profile_sensor_data
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def wait_until(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return
        self.fail("Timed out: " + description)

    def test_imu_world_attachment(self):
        path = os.path.join(os.path.dirname(__file__), "test_simulation.yaml")
        description = meta_description.IMUMetaDescription(path, "robot")
        urdf = ET.fromstring(meta_description.generate_xml_urdf_description_str(
            "simulation_gazebo", description, standalone=True
        ))
        self.assertIsNotNone(urdf.find("link[@name='world']"))
        joint = urdf.find("joint[@name='robot_imu_joint']")
        self.assertIsNotNone(joint)
        self.assertEqual(joint.find("parent").get("link"), "world")
        self.assertEqual(joint.find("child").get("link"), "robot_imu_link")
        origin = joint.find("origin")
        self.assertEqual(list(map(float, origin.get("xyz").split())), [1.0, 2.0, 3.0])
        for actual, degrees in zip(map(float, origin.get("rpy").split()), [4.0, 5.0, 6.0]):
            self.assertAlmostEqual(actual, math.radians(degrees))

    def test_imu_publishes_data(self):
        self.wait_until(lambda: len(self.messages) >= 5, 60, "no IMU data received")
        stamps = []
        for message in self.messages:
            self.assertEqual(message.header.frame_id, "robot_imu_link")
            stamps.append(message.header.stamp.sec * 10**9 + message.header.stamp.nanosec)
            values = [
                message.orientation.x, message.orientation.y,
                message.orientation.z, message.orientation.w,
                message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z,
                message.linear_acceleration.x, message.linear_acceleration.y,
                message.linear_acceleration.z,
            ]
            self.assertTrue(all(math.isfinite(value) for value in values))
            self.assertAlmostEqual(sum(value * value for value in values[:4]), 1.0, delta=0.01)
        self.assertGreater(stamps[0], 0)
        self.assertTrue(all(current > previous for previous, current in zip(stamps, stamps[1:])))
