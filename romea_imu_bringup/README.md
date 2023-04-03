# 1) Overview #

The romea_imu_bringup package provides  : 

 - launch files able to launch ros2 IMU drivers according a meta-description file provided by user (see next section for IMU meta-description file overview), supported drivers are :

   - [bluespace_ai_xsens_mti_driver](https://github.com/bluespace-ai/bluespace_ai_xsens_ros_mti_driver)
   - [xsens_driver](https://github.com/norlab-ulaval/ethzasl_xsens_driver.git)

   It is possible to launch a driver via command line : 

    ```console
    ros2 launch romea_imu_bringup imu_driver.launch.py robot_namespace:=robot meta_description_file_path:=/path_to_file/meta_description_file.yaml
    ```

   where :

   - *robot_namespace* is the name of the robot 
   - *meta_description_file_path* is the absolute path of meta-description file    

 - a python module able to load and parse IMU meta-description file as well as to create URDF description of the IMU sensor a given meta-description.

 - a ros2 python executable able to create IMU URDF description via command line according a given meta-description file  :

  ```console
  ros2 run romea_imu_bringup urdf_description.py robot_namespace:robot meta_description_file_path:/path_to_file/meta_description_file.yaml > imu.urdf`
  ```

   where :

   - *robot_namespace* is the name of the robot 
   - *meta_description_file_path* is the absolute path of meta-description file    

   This URDF  can be directly concatened with mobile base and other sensor URDFs to create a complete URDF description of the robot.  

   



# 2) IMU meta-description #

As seen below IMU meta-description file is a yaml file constituted by five items. The first item is the name of sensor defined by user. The second one is the configuration of ROS2 driver used to control IMU sensor (see section 4 for more explanations). The third item provides basics specifications of the IMU sensor and the fourth item specifies where the IMU sensor is located on the robot, these informations will be used to create URDF description and by user to configure its algorithms.  Finally, the last item gives the topics to be recorded into the ROS bag during experiments or simulation. Thanks to remappings written into launch files, IMU topics are always the same names for each drivers or simulator plugins.       

Example :
```yaml
name: "imu"  #name of the imu
driver: # ros2 driver configuration
  pkg: "xsens_driver" # ros2 driver package choiced by user and its parameters 
  #  pkg: "bluespace_ai_xsens_mti_driver"
  device: "/dev/ttyUSB0" # serial device
  baudrate: 115200 # baudrate
configuration: # imu configuration
  type: xsens # type
  model: mti # model
  rate: 100 # hz
geometry: # geometry configuration 
  parent_link: "base_link" # name of the the parent where the IMU is attached
  xyz: [0.0, 0.0, 1.0] # it's position in meters
  rpy: [0.0, 0.0, 0.0] # it's orientation in degrees
records: #record configuration
  data: true # data topic will be recorded into ros bag
```

# 4) Supported IMU models

Supported IMU are listed in the following table :

|  type  |   model    |
| :----: | :--------: |
| xsens  |    mti     |
| xsens  |   mti6xx   |
| gladiator  |   landmark40_ahrs   |

You can find specifications of each IMU in config directory of romea_imu_description package.

# 5) Supported IMU ROS2 drivers

Supported drivers are [bluespace_ai_xsens_mti_driver](https://github.com/bluespace-ai/bluespace_ai_xsens_ros_mti_driver) and  [xsens_driver](https://github.com/norlab-ulaval/ethzasl_xsens_driver.git). In order to used one of them, you can specify driver item in IMU meta-description file like this:

- Bluespace ai xsens mti_driver:

```yaml
  pkg: "bluespace_ai_xsens_mti_driver"  # ROS2 package name  
    device:  "/dev/ttyUSB0"  # serial device
    baudrate: 115200 # serial baudrate
```

* Xsens driver:

```yaml
  pkg: "bluespace_ai_xsens_mti_driver"  # ROS2  package name  
    device:  "/dev/ttyACM0"  # serial device
    baudrate: 115200 # serial baudrate
```

For each driver a python launch file with the name of the ROS2 package is provided in launch directory. When the meta-description is read by the main launch file called imu_driver.launch.py the corresponding driver is automatically launched taking into account parameters define by user. Thanks to remapping defined inside each driver launch files, the data provided by drivers are always published in the same topic called:

- data(sensors_msgs/IMU)
