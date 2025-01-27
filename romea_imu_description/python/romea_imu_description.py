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


def get_xsens_model_family(model):
    if 'mti' == model:
        return 'mti'
    elif 'mti6' in model:
        return 'mti6xx'

    raise RuntimeError(
        f'Xsens {model} IMU is not supported by romea_imu_description package. '
        'Please check your configuration or contribute to support this sensor.'
    )


def get_gladiator_model_family(model):
    if 'landmark' in model:
        if model.replace("landmark", "") == "40":
            return "landmarkx0"

    raise RuntimeError(
        f'Gladiator {model} IMU is not supported by romea_imu_description package. '
        'Please check your configuration or contribute to support this sensor.'
    )


def get_imu_family(manufacturer, model):
    if manufacturer == 'xsens':
        return get_xsens_model_family(model)
    elif manufacturer == 'gladiator':
        return get_gladiator_model_family(model)
    else:
        raise RuntimeError(
            f"No '{manufacturer}' IMU is supported by romea_imu_description package. "
            'Please check your configuration or contribute to support this kind of sensor.'
        )


def get_imu_specifications_file_path(manufacturer, model):
    family = get_imu_family(manufacturer, model)
    pkg_path = get_package_share_directory('romea_imu_description')
    return f'{pkg_path}/config/{manufacturer}_{family}_specifications.yaml'


def get_imu_specifications(manufacturer, model):
    with open(get_imu_specifications_file_path(manufacturer, model)) as f:
        return yaml.safe_load(f)


def get_imu_geometry_file_path(manufacturer, model):
    family = get_imu_family(manufacturer, model)
    pkg_path = get_package_share_directory('romea_imu_description')
    return f'{pkg_path}/config/{manufacturer}_{family}_geometry.yaml'


def get_imu_geometry(manufacturer, model):
    with open(get_imu_geometry_file_path(manufacturer, model)) as f:
        return yaml.safe_load(f)


def get_imu_specification_units_file_path():
    pkg_path = get_package_share_directory('romea_imu_description')
    return f'{pkg_path}/config/specifications_units.yaml'


def get_imu_specification_units():
    with open(get_imu_specification_units_file_path()) as f:
        return yaml.safe_load(f)


def get_imu_complete_configuration(imu_name, imu_description):

    model = imu_description["model"]
    manufacturer = imu_description["manufacturer"]
    imu_name = f'{manufacturer} {model} imu called {imu_name}'
    specifications = get_imu_specifications(manufacturer, model)
    specifications_units = get_imu_specification_units()

    imu = Device(imu_name, specifications, imu_description, specifications_units)

    configuration = {}
    configuration['rate'] = imu.get('rate')
    configuration['type'] = imu.get('type')

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

    configuration = get_imu_complete_configuration(imu_name, imu_description)

    configuration_yaml_file = f'/tmp/{prefix}{imu_name}_urdf_configuration.yaml'

    with open(configuration_yaml_file, 'w') as f:
        yaml.dump({**configuration, **imu_location}, f)

    geometry_yaml_file = get_imu_geometry_file_path(
        imu_description["manufacturer"], imu_description["model"]
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
