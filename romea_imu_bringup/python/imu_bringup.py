#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
import imu_description
import yaml

def urdf_description(prefix,description_yaml_file):

   with open(description_yaml_file) as f:
     device = yaml.safe_load(f)

   return imu_description.urdf(
       prefix,
       device["name"],
       device["configuration"]["type"],
       device["configuration"]["model"],
       device["configuration"]["rate"],
       device["configuration"]["parent_link"],
       device["configuration"]["xyz"],
       device["configuration"]["rpy"],
   )
