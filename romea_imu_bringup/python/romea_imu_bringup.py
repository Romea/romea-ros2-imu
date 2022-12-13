#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
import romea_imu_description
import yaml


def get_imu_name(meta_description):
  return meta_description["name"]

def has_imu_driver_configuration(meta_description):
  return "driver" in meta_description

def get_imu_driver_pkg(meta_description):
  return meta_description["driver"]["pkg"]

def get_imu_device(meta_description):
  return meta_description["driver"]["device"]

def get_imu_baudrate(meta_description):
  return meta_description["driver"]["baudrate"]

def get_imu_type(meta_description):
  return meta_description["configuration"]["type"]

def get_imu_model(meta_description):
  return meta_description["configuration"]["model"]

def get_imu_rate(meta_description):
  return meta_description["configuration"]["rate"]

def get_imu_parent_link(meta_description):
  return meta_description["geometry"]["parent_link"]

def get_imu_xyz(meta_description):
  return meta_description["geometry"]["xyz"]

def get_imu_rpy(meta_description):
  return meta_description["geometry"]["rpy"]


def urdf_description(prefix,meta_description_filename):

   with open(meta_description_filename) as f:
     meta_description = yaml.safe_load(f)

   return romea_imu_description.urdf(
       prefix,
       get_imu_name(meta_description),
       get_imu_type(meta_description),
       get_imu_model(meta_description),
       get_imu_rate(meta_description),
       get_imu_parent_link(meta_description),
       get_imu_xyz(meta_description),
       get_imu_rpy(meta_description),
   )
