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
import romea_common_description
from ament_index_python.packages import get_package_share_directory


def get_specifications_file_path(imu_description):
    return romea_common_description.get_specifications_file_path(
        "romea_imu_description", imu_description)


def get_specifications(imu_description):
    with open(get_specifications_file_path(imu_description)) as f:
        return yaml.safe_load(f)


def get_geometry_file_path(imu_description):
    return romea_common_description.get_geometry_file_path(
        "romea_imu_description", imu_description)


def get_geometry(imu_description):
    with open(get_geometry_file_path(imu_description)) as f:
        return yaml.safe_load(f)


def get_specification_units_file_path():
    pkg_path = get_package_share_directory('romea_imu_description')
    return f'{pkg_path}/config/specifications_units.yaml'


def get_specification_units():
    with open(get_specification_units_file_path()) as f:
        return yaml.safe_load(f)


def get_complete_configuration(imu_name, imu_description, imu_location):

    model = imu_description["model"]
    version = imu_description["version"]
    manufacturer = imu_description["manufacturer"]
    imu_name = f'{manufacturer} {model} {version} imu called {imu_name}'
    specifications = get_specifications(imu_description)
    specifications_units = get_specification_units()

    imu = romea_common_description.DeviceConfiguration(
        imu_name, specifications, imu_description, specifications_units
    )

    configuration = {}
    configuration["model"] = imu_description["model"]
    configuration["version"] = imu_description["version"]
    configuration["manufacturer"] = imu_description["manufacturer"]
    configuration['type'] = imu.get('type')
    configuration['rate'] = imu.get('rate')
    configuration['acceleration_noise_density'] = imu.get('acceleration_noise_density')
    configuration['acceleration_bias_stability_std'] = imu.get('acceleration_bias_stability_std')
    configuration['acceleration_range'] = imu.get('acceleration_range')
    configuration['angular_speed_noise_density'] = imu.get('angular_speed_noise_density')
    configuration['angular_speed_bias_stability_std'] = imu.get('angular_speed_bias_stability_std')
    configuration['angular_speed_range'] = imu.get('angular_speed_range')
    if configuration['type'] == "AHRS":
        configuration['magnetic_noise_density'] = imu.get('magnetic_noise_density')
        configuration['magnetic_bias_stability_std'] = imu.get('magnetic_bias_stability_std')
        configuration['magnetic_range'] = imu.get('magnetic_range')
        configuration['heading_std'] = imu.get('heading_std')

    return {**configuration, **imu_location}


def generate_configuration_file(configuration, extended):
    units = get_specification_units()
    return romea_common_description.generate_configuration_file(configuration, units, extended)


def generate_urdf_description(
        prefix, mode, imu_name, imu_description, imu_location, ros_namespace
):

    configuration = get_complete_configuration(imu_name, imu_description, imu_location)

    configuration_yaml_file = f'/tmp/{prefix}{imu_name}_configuration.yaml'
    with open(configuration_yaml_file, 'w') as f:
        f.write(generate_configuration_file(configuration, False))

    geometry_yaml_file = get_geometry_file_path(imu_description)

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
