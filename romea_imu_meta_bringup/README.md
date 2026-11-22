# romea_imu_meta_bringup

## 1) Overview

romea_imu_meta_bringup provides tools to describe and launch IMU sensor using a meta-description approach.

It allows defining IMU in a high-level YAML format and automatically generating consistent ROS2 artifacts such as:

* configuration files → used as generic ROS2 configuration inputs
* launch files → used to start IMU drivers and/or bridge for simulation
* URDF description files → used to load the imu into simulators

This package is built on top of `romea_common_meta_bringup` and specializes it for IMU sensor integration. 

It also provides launch files that allow controlling the IMU both on a real robot and in simulation.

---

## 2) IMU meta-description concept

An `imu meta-description` is a YAML file that defines an IMU sensor and how it should be integrated into a system. It centralizes:

* IMU identification (name, namespace)
* hardware configuration (manufacturer, model, version)
* kinematic attachment (parent link, pose)
* ROS2 launch description

---

### Example meta-description

```yaml id="9lfc1h"
name: imu
namespace: ns
launch:
  - include:
      file: "$(find-pkg-share romea_imu_meta_bringup)/profile/xsens_driver.launch.py"
      arg:
        - name: device
          value: /dev/ttyUSB0
        - name: baudrate
          value: "115200"
      if: $(eval "'$(var mode)' == 'live'")
  - include:
      file: $(find-pkg-share romea_imu_meta_bringup)/profile/gz_bridge.launch.py
      arg:
        - name: container
          value: /gz_container 
      if: $(eval "'$(var mode)' == 'simulation_gazebo'")
configuration:
  manufacturer: xsens
  model: mti
  rate: 100
location:
  parent_link: "base_link"
  xyz: [1.0, 2.0, 3.0]
  rpy: [4.0, 5.0, 6.0]
records:
  data: true
```

### Launch files Profiles

The `profile/` directory contains reusable ROS2 launch files dedicated to IMU bringup.

These launch profiles provide predefined setups for common execution contexts, such as:

* starting a real IMU driver, for example xsens_driver.launch.py
* starting a simulation bridge, for example gz_bridge.launch.py
* starting localization plugin that extract observations from IMU
* reusing standardized bringup configurations across live and simulation modes

Each profile is intended to be included from the launch section of an IMU meta-description. This makes it possible to select the appropriate runtime behavior depending on the selected mode, while keeping the meta-description concise and consistent.

## 3) Scripts

`romea_imu_meta_bringup` provides several scripts to generate ROS2 artifacts (configuration, launch and URDF files) from an IMU meta-description; the usage and resulting outputs are described below.

### Generate configuration file

```bash id="rq0ybi"
generate-imu-configuration-file \
  meta_description_file_path:=path/to/imu_meta_description.yaml \
  extended:=false
```

#### Example output

```yaml id="pgd8yu"
model: mti
version: 
manufacturer: xsens
type: AHRS
rate: 100  # unit Hz
acceleration_noise_density: 0.0005  # unit m/s^2/√Hz
acceleration_bias_stability_std: 0.02  # unit m/s^2
acceleration_range: 50.0  # unit m/s^2
angular_speed_noise_density: 0.05  # unit °/s/√Hz
angular_speed_bias_stability_std: 20  # unit °/s
angular_speed_range: 300  # unit °/s
magnetic_noise_density: 5e-08  # unit T/√Hz
magnetic_bias_stability_std: 1e-08  # unit T
magnetic_range: 7.5e-05  # unit T
heading_std: 1  # unit °
parent_link: base_link
xyz: [1.0, 2.0, 3.0]  # unit m
rpy: [4.0, 5.0, 6.0]  # unit °
```
---

### Generate URDF Description

Generates the IMU URDF description from the meta-description.

```bash 
generate-imu-urdf-description-file \
  robot_namespace:=robot \
  meta_description_file_path:=path/to/imu_meta_description.yaml \
  mode:=simulation_gazebo
```

The generated URDF description defines how the IMU is attached to the robot model. It includes the IMU link, the fixed joint between the parent link and the IMU frame, and, in simulation mode, the Gazebo sensor description.


#### Example output when using gazebo simulation

```xml
<link name="robot_imu_link">
  ...
</link>

<joint name="robot_imu_joint" type="fixed">
  <origin xyz="1.0 2.0 3.0"
          rpy="0.06981317007977318 0.08726646259971647 0.10471975511965977"/>
  <parent link="robot_base_link"/>
  <child link="robot_imu_link"/>
</joint>

<gazebo reference="robot_imu_link">
  <sensor name="robot_imu" type="imu">
    <update_rate>100</update_rate>
    <topic>/robot/ns/imu/data</topic>
    <gz_frame_id>robot_imu_link</gz_frame_id>
    ...
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
  </sensor>
</gazebo>
```

This URDF description can then be loaded into the robot description and used by Gazebo to publish simulated IMU measurements. 

It can also be directly concatenated with mobile base and other device URDF descriptions to build a complete robot model.

### Generate launch file

Generates a YAML ROS2 launch file from the meta-description.

```bash
generate-imu-launch-file \
  robot_namespace:=robot \
  meta_description_file_path:=path/to/imu_meta_description.yaml
```

#### Example output

```yaml
launch:
- arg: {name: mode, default: live}
- group:
  - push-ros-namespace: {namespace: robot}
  - push-ros-namespace: {namespace: ns}
  - push-ros-namespace: {namespace: imu}
  - let: {name: model, value: mti}
  - let: {name: version, value: ''}
  - let: {name: manufacturer, value: xsens}
  - let: {name: type, value: AHRS}
  - let: {name: rate, value: '100'}
  - let: {name: acceleration_noise_density, value: '0.0005'}
  - let: {name: acceleration_bias_stability_std, value: '0.02'}
  - let: {name: acceleration_range, value: '50.0'}
  - let: {name: angular_speed_noise_density, value: '0.05'}
  - let: {name: angular_speed_bias_stability_std, value: '20'}
  - let: {name: angular_speed_range, value: '300'}
  - let: {name: magnetic_noise_density, value: 5e-08}
  - let: {name: magnetic_bias_stability_std, value: 1e-08}
  - let: {name: magnetic_range, value: '7.5e-05'}
  - let: {name: heading_std, value: '1'}
  - let: {name: parent_link, value: base_link}
  - let: {name: xyz, value: '[1.0, 2.0, 3.0]'}
  - let: {name: rpy, value: '[4.0, 5.0, 6.0]'}
  - let: {name: tf_prefix, value: robot_}
  - let: {name: frame_id, value: robot_imu_link}
  - include:
      file: $(find-pkg-share romea_imu_meta_bringup)/profile/xsens_driver.launch.py
      arg: [{name: device, value: /dev/ttyUSB0}, {name: baudrate, value: '115200'}]
      if: $(eval "'$(var mode)' == 'live'")
  - include:
      file: $(find-pkg-share romea_imu_meta_bringup)/profile/gz_bridge.launch.py
      arg: [{name: container, value: /gz_container}]
      if: $(eval "'$(var mode)' == 'simulation_gazebo'")
```

#### Notes

* the launch file is generated from the `launch` section of the meta-description
* namespaces are automatically constructed (`robot → device → imu`)
* all configuration values are exposed as `let` variables
* the selected profiles are included at the end

This file can be used directly with ROS2 or generated dynamically using `imu.launch.py`.

## 4) Usage

The package provides **two main launch files**:

* `imu.launch.py` → for dynamic bringup (live or simulation mode)
* `simulation.launch.py` → for full simulation test

---

### Dynamic bringup

When using `imu.launch.py`, the following steps are performed automatically:

```bash
ros2 launch romea_imu_meta_bringup imu.launch.py \
  mode:=live \
  robot_namespace:=robot \
  meta_description_file_path:=path/to/imu_meta_description.yaml
```

* generation of the URDF description
* generation of the imu configuration file
* generation of the launch file
* execution of the generated launch file


#### Live or simulation mode

The `mode` parameter controls the behavior:

* `live` → starts the imu driver and controllers
* `simulation_<simulator>` → starts simulation bridge

---

### Simulation test

For a complete simulation setup, use:

```bash
ros2 launch romea_imu_meta_bringup simulation.launch.py \
  simulator_type:=gazebo \
  robot_namespace:=robot \
  meta_description_file_path:=path/to/imu_meta_description.yaml
```

This launch file:

* starts the simulator
* generates and loads the URDF
* spawns the imu in simulation
* calls `imu.launch.py` to start the gazebo bridge

Pass `gui:=false` to run without the simulator GUI.

---
## 5) Supported imus

Currently, the following IMU manufacturers and models are supported::

|  manufacturer  |   model    | version    |
| :----: | :--------: | :--------: |
| xsens  |   mti   |  1, 6xx |
| gladiator  |   landmark | 40_ahrs   |

Details and specifications for each model can be found in the config directory of the `romea_imu_description` package.



## 6) Supported IMU ROS2 Drivers

The package currently supports the following ROS2 IMU drivers:

* bluespace_ai_xsens_mti_driver
* xsens_driver

Dedicated launch profiles are provided in the profile/ directory to start each supported driver through a standardized interface.

These profiles can be included directly from the launch section of the IMU meta-description.

#### Example using bluespace_ai_xsens_mti_driver

```yaml
launch:
  - include:
      file: "$(find-pkg-share romea_imu_meta_bringup)/profile/bluespace_ai_xsens_mti_driver.launch.py"
      arg:
        - name: device
          value: /dev/ttyUSB0
        - name: baudrate
          value: "115200"
```

#### Example using xsens_driver

```yaml
launch:
  - include:
      file: "$(find-pkg-share romea_imu_meta_bringup)/profile/xsens_driver.launch.py"
      arg:
        - name: device
          value: /dev/ttyACM0
        - name: baudrate
          value: "115200"
```

Each launch profile is responsible for:

* starting the corresponding driver node
* configuring driver parameters
* applying standardized topic remappings

This abstraction ensures that all supported IMU drivers expose a consistent ROS2 interface independently of their internal implementation.
