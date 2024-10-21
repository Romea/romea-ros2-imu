# ROMEA IMU Bringup #

# 1) Overview #

The romea_imu_bringup package provides  : 

 - **Launch files** able to launch ros2 IMU drivers according a meta-description file provided by user (see next section for IMU meta-description file overview), supported drivers are:

   - [bluespace_ai_xsens_mti_driver](https://github.com/bluespace-ai/bluespace_ai_xsens_ros_mti_driver)
   - [xsens_driver](https://github.com/norlab-ulaval/ethzasl_xsens_driver.git)

   You can launch a driver using the following command: 

    ```console
    ros2 launch romea_imu_bringup imu_driver.launch.py robot_namespace:=robot meta_description_file_path:=/path_to_file/meta_description_file.yaml
    ```

   where:

   - *robot_namespace* is the name of the robot 
   - *meta_description_file_path* is the absolute path of meta-description file    

 - A **Python module** able to load and parse IMU meta-description file as well as to create URDF description of the IMU sensor a given meta-description.

 - A **ROS2 python executable** able to create IMU URDF description via command line according a given meta-description file:

  ```console
  ros2 run romea_imu_bringup urdf_description.py robot_namespace:robot meta_description_file_path:/path_to_file/meta_description_file.yaml > imu.urdf`
  ```

   where:

   - *robot_namespace* is the name of the robot 
   - *meta_description_file_path* is the absolute path of meta-description file    

   This URDF  can be directly concatened with mobile base and other sensor URDFs to create a complete URDF description of the robot.  

   



# 2) IMU meta-description #

The IMU meta-description file is a YAML file containing five key elements:
- **name**: A user-defined name for the IMU sensor.
- **driver**: Specifies the ROS2 driver and its parameters (detailed in Section 5).
- **configuration**: Describes basic specifications of the IMU.
- **geometry**: Describes the position and orientation of the IMU on the robot, used for URDF generation.
- **records**: Specifies which topics should be recorded during experiments or simulations.

Example:
```yaml
name: imu  #name of the imu
driver: # ros2 driver configuration
  pkg: xsens_driver # ros2 driver package choiced by user and its parameters 
  executable: mtnode.py  # node to be launch
  parameters: # driver node parameters
    device: /dev/ttyUSB0 # serial device
    baudrate: 115200 # baudrate
configuration: # imu configuration
  type: xsens # type
  model: mti # model
  rate: 100 # hz
geometry: # geometry configuration 
  parent_link: base_link # name of the the parent where the IMU is attached
  xyz: [0.0, 0.0, 1.0] # it's position in meters
  rpy: [0.0, 0.0, 0.0] # it's orientation in degrees
records: #record configuration
  data: true # data topic will be recorded into ros bag
```

# 4) Supported IMU models

Supported IMU types and models include:

|  type  |   model    |
| :----: | :--------: |
| xsens  |    mti     |
| xsens  |   mti6xx   |
| gladiator  |   landmark40_ahrs   |

Details and specifications for each model can be found in the config directory of the romea_imu_description package.

# 5) Supported IMU ROS2 drivers

Supported drivers are [bluespace_ai_xsens_mti_driver](https://github.com/bluespace-ai/bluespace_ai_xsens_ros_mti_driver) and  [xsens_driver](https://github.com/norlab-ulaval/ethzasl_xsens_driver.git). In order to used one of them, you can specify driver item in IMU meta-description file like this:

- **Bluespace ai xsens mti_driver**:

```yaml
  pkg: "bluespace_ai_xsens_mti_driver"  # ROS2 package name  
  executable: xsens_mti_node  # node to be launch
  parameters: # node parameters
    device:  "/dev/ttyUSB0"  # serial device
    baudrate: 115200 # serial baudrate
```

- **Xsens driver**:

```yaml
  pkg: "bluespace_ai_xsens_mti_driver"  # ROS2  package name  
  executable: mtnode.py  # node to be launch
  parameters: # node parameters
    device:  "/dev/ttyACM0"  # serial device
    baudrate: 115200 # serial baudrate
```

A dedicated Python launch file is provided for each driver in the launch directory. When you launch the imu_driver.launch.py file, the corresponding driver node will automatically launch, using the parameters defined by the user. Thanks to remapping defined inside each driver launch files, the data provided by drivers are always published in the same topic called:

- data(sensors_msgs/IMU)
