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
from romea_common_description import get_configuration_file
from romea_common_description import generate_configuration_file
from romea_common_description import DeviceConfiguration as Device
from ament_index_python.packages import get_package_share_directory


def get_imu_configuration_file_path(manufacturer, model, version, what):
    package = get_package_share_directory('romea_imu_description')
    configuration_file_path = get_configuration_file(
        f"{package}/config", f"{manufacturer}_{model}_{version}_{what}.yaml"
    )

    if not configuration_file_path:
        raise RuntimeError(
            f"No {manufacturer} {model} {version} IMU is supported by {package} package. "
            'Please check your configuration or contribute to support this kind of sensor.'
        )

    return configuration_file_path


def get_imu_specifications_file_path(manufacturer, model, version):
    return get_imu_configuration_file_path(manufacturer, model, version, "specifications")


def get_imu_specifications(manufacturer, model, version):
    with open(get_imu_specifications_file_path(manufacturer, model, version)) as f:
        return yaml.safe_load(f)


def get_imu_geometry_file_path(manufacturer, model, version):
    return get_imu_configuration_file_path(manufacturer, model, version, "geometry")


def get_imu_geometry(manufacturer, model, version):
    with open(get_imu_geometry_file_path(manufacturer, model, version)) as f:
        return yaml.safe_load(f)


def get_imu_specification_units_file_path():
    pkg_path = get_package_share_directory('romea_imu_description')
    return f'{pkg_path}/config/specifications_units.yaml'


def get_imu_specification_units():
    with open(get_imu_specification_units_file_path()) as f:
        return yaml.safe_load(f)


def get_imu_complete_configuration(imu_name, imu_description):

    model = imu_description["model"]
    version = imu_description["version"]
    manufacturer = imu_description["manufacturer"]
    imu_name = f'{manufacturer} {model} {version} imu called {imu_name}'
    specifications = get_imu_specifications(manufacturer, model, version)
    specifications_units = get_imu_specification_units()

    imu = Device(imu_name, specifications, imu_description, specifications_units)

    configuration = {}
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

    return configuration


def urdf(prefix, mode, imu_name, imu_description, imu_location, ros_namespace):

    units = get_imu_specification_units()
    configuration = get_imu_complete_configuration(imu_name, imu_description)
    configuration_yaml_file = f'/tmp/{prefix}{imu_name}_urdf_configuration.yaml'

    with open(configuration_yaml_file, 'w') as f:
        f.write(generate_configuration_file({**configuration, **imu_location}, units, False))

    geometry_yaml_file = get_imu_geometry_file_path(
        imu_description["manufacturer"], imu_description["model"], imu_description["version"]
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
