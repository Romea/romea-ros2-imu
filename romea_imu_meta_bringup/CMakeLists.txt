cmake_minimum_required(VERSION 3.5)
project(romea_imu_meta_bringup)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_module(python/romea_imu_meta_bringup.py)

install(PROGRAMS
  scripts/urdf_description.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  configure_file(test/test_imu_meta_bringup.yaml test_imu_meta_bringup.yaml)
  configure_file(test/test_xsens_driver_profile.yaml test_xsens_driver_profile.yaml)
  configure_file(test/test_bluespace_ai_xsens_mti_driver_profile.yaml test_bluespace_ai_xsens_mti_driver_profile.yaml)
  ament_add_pytest_test(test_imu_meta_bringup test/test_imu_meta_bringup.py)
  ament_add_pytest_test(test_urdf_description test/test_urdf_description.py)
  ament_add_pytest_test(test_imu_driver_parameters test/test_imu_driver_configuration.py)
endif()

ament_package()
