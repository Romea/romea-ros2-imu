# romea_ros2_imu

## Overview

`romea_ros2_imu` groups the ROS2 packages used to describe, launch and simulate IMU, VRU and AHRS sensors in the ROMEA ecosystem.

This repository-level README gives a map of the stack. Detailed behavior, configuration formats, generated files and launch examples are documented in the README of each package listed below.

## Packages

| Package | Role |
| --- | --- |
| `romea_imu` | Metapackage for the IMU stack. |
| `romea_imu_description` | IMU specifications, geometry files, Python helpers and URDF generation. |
| `romea_imu_meta_bringup` | Main user entry point: IMU meta-description parser, launch generation, URDF generation and reusable launch profiles. |

## Usage

This stack is usually consumed from a larger ROMEA workspace or from a demo configuration that already selects the IMU devices to launch.

In most cases, start with `romea_imu_meta_bringup`. It is the user-facing entry point of the stack: from an IMU meta-description, it can generate the detailed IMU configuration, generate the URDF fragment, and launch the selected live driver, simulation bridge or localisation plugin. `romea_imu_description` provides the sensor description data used behind this entry point.

Use the specialized package README files when you need to inspect or extend a specific part of the stack:

* `romea_imu_description` to add or inspect IMU specifications and geometry;
* `romea_imu_meta_bringup` to write IMU meta-descriptions and launch profiles.

## License

This project is released under the Apache License 2.0. See the `LICENSE` file for details.

## Authors

The `romea_ros2_imu` stack was developed by Jean Laneurit in the context of research projects carried out at INRAE.

## Contact

For questions or comments about this stack, contact [Jean Laneurit](mailto:jean.laneurit@inrae.fr).
