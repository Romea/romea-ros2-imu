# romea_imu_description

## Overview

`romea_imu_description` provides URDF descriptions and configuration utilities for IMU sensors using a **specification-based approach**.

It allows defining IMU characteristics and generating consistent ROS2 artifacts using the provided Python module, such as:

* configuration files
* URDF description files

This package is designed to be used together with `romea_imu_meta_bringup`.

---

## Imu description concept

An IMU is described using two inputs:

* **IMU description**: defines hardware characteristics (manufacturer, model, version, rate)
* **IMU location**: defines how the IMU is attached to the robot (parent link, pose)

These inputs are combined with IMU specifications to build a complete configuration used to generate ROS2 artifacts.

---

### Example

```yaml
imu_description:
  manufacturer: xsens
  model: mti
  version: "640"
  rate: 100

imu_location:
  parent_link: base_link
  xyz: [1.0, 2.0, 3.0] #m
  rpy: [4.0, 5.0, 6.0] #°
```

---

### Notes

IMU specifications are defined in files located in the `config/` directory and follow the pattern `<manufacturer>_<model>.<version>_specifications.yaml` (e.g. `xsens_mti_xxx_specifications.yaml`). These specification files provide default values such as update rate, noise ans biaises charadteristics, measurement ranges, which can be overridden by user-defined values in `imu_description`. 

The `xyz` and `rpy` fields define the pose of the IMU relative to the parent link. The `xyz` values specify the translation (in meters), while `rpy` defines the orientation using roll, pitch, and yaw angles expressed in degrees for readability and ease of definition by users. These angles are automatically converted to radians internally by the scripts to comply with ROS and URDF conventions. Together, they describe how the IMU is positioned and oriented within the robot frame.

---

## Python API

The package provides utilities to generate configuration, controllers configuration and URDF descriptions from an IMU description.

---

### get_complete_configuration

Builds a complete imu configuration by combining:

* IMU description
* IMU location
* IMU specifications

---

### generate_configuration_file_str

Generates the IMU configuration file as a YAML string from the configuration returned by get_complete_configuration.

---

### generate_urdf_description_str

Generates the IMU URDF description as a string, ready to be written to a URDF file. The URDF description is generated from the complete configuration returned by `get_complete_configuration` and from an associated geometry configuration file. The generated URDF is used by simulators and visualization tools.

---

## Example

```python
from romea_imu_description import (
    get_complete_configuration,
    generate_configuration_file_str,
    generate_urdf_description_str,
)

imu_name = "imu"

imu_description = {
  manufacturer: xsens
  model: mti
  version: "640"
  rate: 100
}

imu_location = {
    "parent_link": "base_link",
    "xyz": [1.0, 2.0, 3.0],
    "rpy": [4.0, 5.0, 6.0],
}

prefix = "robot_"
ros_namespace = "/robot/imu"
mode = "live"

configuration = get_complete_configuration(
    imu_name,
    imu_description,
    imu_location,
)

configuration_yaml = generate_configuration_file_str(configuration)

urdf_description = generate_urdf_description_str(
    prefix,
    mode,
    imu_name,
    imu_description,
    imu_location,
    ros_namespace,
)
```

The returned values can be written to files if needed.

---

## Usage

This package is typically used together with:

* `romea_imu_meta_bringup` → generates launch files and handles bringup

---

## Supported imus

Currently, the package supports the following IMU manufacturers and models:

| Manufacturer | Model | Version |
|:------------:|:-----:|:-------:|
| xsens | mti | 1, 6xx |
| gladiator | landmark | 40_ahrs |

Support for additional IMU models may be added in future releases.