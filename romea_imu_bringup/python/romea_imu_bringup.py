#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from romea_common_bringup import MetaDescription
import romea_imu_description
import yaml


class IMUMetaDescription:
    def __init__(self, meta_description_filename):
        self.meta_description = MetaDescription("imu", meta_description_filename)

    def get_name(self):
        return self.meta_description.get("name")

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
        return self.meta_description.get("rate","configuration")

    def get_parent_link(self):
        return self.meta_description.get("parent_link", "geometry")

    def get_xyz(self):
        return self.meta_description.get("xyz", "geometry")

    def get_rpy(self):
        return self.meta_description.get("rpy", "geometry")

# def get_imu_name(meta_description):
#   return meta_description["name"]

# def has_imu_driver_configuration(meta_description):
#   return "driver" in meta_description

# def get_imu_driver_pkg(meta_description):
#   return meta_description["driver"]["pkg"]

# def get_imu_device(meta_description):
#   return meta_description["driver"]["device"]

# def get_imu_baudrate(meta_description):
#   return meta_description["driver"]["baudrate"]

# def get_imu_type(meta_description):
#   return meta_description["configuration"]["type"]

# def get_imu_model(meta_description):
#   return meta_description["configuration"]["model"]

# def get_imu_rate(meta_description):
#   return meta_description["configuration"]["rate"]

# def get_imu_parent_link(meta_description):
#   return meta_description["geometry"]["parent_link"]

# def get_imu_xyz(meta_description):
#   return meta_description["geometry"]["xyz"]

# def get_imu_rpy(meta_description):
#   return meta_description["geometry"]["rpy"]


def urdf_description(prefix,meta_description_filename):

   meta_description = IMUMetaDescription(meta_description_filename)

   return romea_imu_description.urdf(
       prefix,
       meta_description.get_name(),
       meta_description.get_type(),
       meta_description.get_model(),
       meta_description.get_rate(),
       meta_description.get_parent_link(),
       meta_description.get_xyz(),
       meta_description.get_rpy(),
   )
