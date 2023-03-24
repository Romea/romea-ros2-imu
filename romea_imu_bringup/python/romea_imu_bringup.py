# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license

from numpy import radians
from romea_common_bringup import MetaDescription, robot_urdf_prefix, device_namespace
import romea_imu_description


class IMUMetaDescription:
    def __init__(self, meta_description_file_path):
        self.meta_description = MetaDescription("imu", meta_description_file_path)

    def get_name(self):
        return self.meta_description.get("name")

    def get_namespace(self):
        return self.meta_description.get_or("namespace", None)

    def has_driver_configuration(self):
        return self.meta_description.exists("driver")

    def get_driver_pkg(self):
        return self.meta_description.get("pkg", "driver")

    def get_driver_device(self):
        return self.meta_description.get("device", "driver")

    def get_driver_baudrate(self):
        return self.meta_description.get("baudrate", "driver")

    def get_type(self):
        return self.meta_description.get("type", "configuration")

    def get_model(self):
        return self.meta_description.get("model", "configuration")

    def get_rate(self):
        return self.meta_description.get("rate", "configuration")

    def get_parent_link(self):
        return self.meta_description.get("parent_link", "geometry")

    def get_xyz(self):
        return self.meta_description.get("xyz", "geometry")

    def get_rpy_deg(self):
        return self.meta_description.get("rpy", "geometry")

    def get_rpy_rad(self):
        return radians(self.get_rpy_deg()).tolist()


def urdf_description(robot_namespace, meta_description_file_path):

    meta_description = IMUMetaDescription(meta_description_file_path)

    ros_namespace = device_namespace(
        robot_namespace,
        meta_description.get_namespace(),
        meta_description.get_name()
    )

    return romea_imu_description.urdf(
        robot_urdf_prefix(robot_namespace),
        meta_description.get_name(),
        meta_description.get_type(),
        meta_description.get_model(),
        meta_description.get_rate(),
        meta_description.get_parent_link(),
        meta_description.get_xyz(),
        meta_description.get_rpy_rad(),
        ros_namespace,
    )
