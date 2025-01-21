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


import xacro
import yaml
from romea_common_description import DeviceConfiguration as Device
from ament_index_python.packages import get_package_share_directory


def get_imu_specifications_file_path(type, model):
    pkg_path = get_package_share_directory('romea_imu_description')
    return f'{pkg_path}/config/{type}_{model}_specifications.yaml'


def get_imu_specifications(type, model):
    with open(get_imu_specifications_file_path(type, model)) as f:
        return yaml.safe_load(f)


def get_imu_geometry_file_path(type, model):
    pkg_path = get_package_share_directory('romea_imu_description')
    return f'{pkg_path}/config/{type}_{model}_geometry.yaml'


def get_imu_geometry(type, model):
    with open(get_imu_geometry_file_path(type, model)) as f:
        return yaml.safe_load(f)


def get_imu_specification_units_file_path():
    pkg_path = get_package_share_directory('romea_imu_description')
    return f'{pkg_path}/config/specifications_units.yaml'


def get_imu_specification_units():
    with open(get_imu_specification_units_file_path()) as f:
        return yaml.safe_load(f)


def get_imu_complete_configuration(imu_name, imu_description):

    type = imu_description["type"]
    model = imu_description["model"]
    lidar_name = f'{type} {model} lidar called {imu_name}'
    specifications = get_imu_specifications(type, model)
    specifications_units = get_imu_specification_units()

    imu = Device(lidar_name, specifications, imu_description, specifications_units)

    configuration = {}
    configuration['rate'] = imu.get('rate')

    configuration['acceleration_noise_density'] = imu.get('acceleration_noise_density')
    configuration['acceleration_bias_stability_std'] = imu.get('maximal_azimut_angle')
    configuration['acceleration_range'] = imu.get('acceleration_range')

    configuration['angular_speed_noise_density'] = imu.get('angular_speed_noise_density')
    configuration['angular_speed_bias_stability_std'] = imu.get('angular_speed_bias_stability_std')
    configuration['angular_speed_range'] = imu.get('angular_speed_range')

    if specifications["type"] == "AHRS":
        configuration['magnetic_noise_density'] = imu.get('magnetic_noise_density')
        configuration['magnetic_bias_stability_std'] = imu.get('magnetic_bias_stability_std')
        configuration['magnetic_range'] = imu.get('magnetic_range')
        configuration['heading_std'] = imu.get('heading_std')

    return configuration


def urdf(prefix, mode, imu_name, imu_description, imu_location, ros_namespace):

    configuration = get_imu_complete_configuration(imu_location, imu_description)

    configuration_yaml_file = f'/tmp/{prefix}{imu_name}_urdf_configuration.yaml'

    with open(configuration_yaml_file, 'w') as f:
        yaml.dump({**configuration, **imu_location}, f)

    geometry_yaml_file = get_imu_geometry_file_path(
        imu_description["type"], imu_description["model"]
    )

    xacro_file = get_package_share_directory("romea_imu_description") + "/urdf/imu.xacro.urdf"

    if mode == "simulation":
        mode += "_gazebo_classic"

    urdf_xml = xacro.process_file(
        xacro_file,
        mappings={
            'prefix': prefix,
            'mode': mode,
            'name': imu_name,
            'sensor_config_yaml_file': configuration_yaml_file,
            'geometry_config_yaml_file': geometry_yaml_file,
            'mesh_visual': str(True),
            'ros_namespace': ros_namespace,
        },
    )

    return urdf_xml.toprettyxml()


# def urdf(prefix, mode, name, type, model, rate, parent_link, xyz, rpy, ros_namespace):

#     xacro_file = get_package_share_directory("romea_imu_description") + "/urdf/imu.xacro.urdf"

#     if mode == "simulation":
#         mode += "_gazebo_classic"

#     urdf_xml = xacro.process_file(
#         xacro_file,
#         mappings={
#             "prefix": prefix,
#             "mode": mode,
#             "name": name,
#             "rate": str(rate),
#             "specifications_yaml_file": get_imu_specifications_file_path(type, model),
#             "geometry_yaml_file": get_imu_geometry_file_path(type, model),
#             "parent_link": parent_link,
#             "xyz": " ".join(map(str, xyz)),
#             "rpy": " ".join(map(str, rpy)),
#             "mesh_visual": str(False),
#             "ros_namespace": ros_namespace,
#         },
#     )

#     return urdf_xml.toprettyxml()
