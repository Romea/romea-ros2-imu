cmake_minimum_required(VERSION 3.5)
project(romea_imu_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY config urdf launch
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_module(python/romea_imu_description.py)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_flake8_CONFIG_FILE "${CMAKE_SOURCE_DIR}/../.flake8")
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_imu_urdf_gazebo_classic test/test_imu_urdf_gazebo_classic.py)
  ament_add_pytest_test(test_xsens_mti_description test/test_xsens_mti_description.py)
  ament_add_pytest_test(test_xsens_mti6xx_description test/test_xsens_mti6xx_description.py)
  ament_add_pytest_test(test_gladiator_landmarkx0_description test/test_gladiator_landmarkx0_description.py)
endif()

ament_package()
