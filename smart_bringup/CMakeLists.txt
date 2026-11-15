cmake_minimum_required(VERSION 3.8)
project(smart_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(imu_filter_madgwick REQUIRED)

add_library(smart_base_protocol src/protocol.cpp)
target_include_directories(smart_base_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(smart_base_node src/smart_base_node.cpp)
target_include_directories(smart_base_node PRIVATE include)
ament_target_dependencies(smart_base_node
  diagnostic_msgs
  diagnostic_updater
  geometry_msgs
  nav_msgs
  rclcpp
  rcl_interfaces
  sensor_msgs
  std_msgs
  tf2_ros
)
target_link_libraries(smart_base_node smart_base_protocol Threads::Threads)

add_executable(smart_imu_filter_node src/smart_imu_filter_node.cpp)
target_include_directories(smart_imu_filter_node PRIVATE include)
target_compile_features(smart_imu_filter_node PRIVATE cxx_std_17)
ament_target_dependencies(smart_imu_filter_node
  imu_filter_madgwick
  rclcpp
  rcl_interfaces
  sensor_msgs
)

install(TARGETS
  smart_base_protocol
  smart_base_node
  smart_imu_filter_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  rviz
  scripts
  udev
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(PROGRAMS scripts/imu_console.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME imu_console
)
install(FILES scripts/imu_attitude.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES LICENSE.imu_fusion
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_flake8)
  set(AMENT_LINT_AUTO_FILE_EXCLUDE doc)
  set(ament_cmake_uncrustify_ADDITIONAL_EXCLUDE "doc/*")
  ament_lint_auto_find_test_dependencies()
  ament_flake8(launch scripts test)

  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol smart_base_protocol)
  target_include_directories(test_protocol PRIVATE include)

  ament_add_gtest(test_imu_fusion test/test_imu_fusion.cpp)
  target_include_directories(test_imu_fusion PRIVATE include)
  target_compile_features(test_imu_fusion PRIVATE cxx_std_17)
  ament_target_dependencies(test_imu_fusion imu_filter_madgwick)

  ament_add_pytest_test(test_smart_base_pty test/test_smart_base_pty.py
    TIMEOUT 60
  )
  ament_add_pytest_test(test_scan_geometry test/test_scan_geometry.py
    TIMEOUT 30
  )
  ament_add_pytest_test(test_udev_installer test/test_udev_installer.py
    TIMEOUT 30
  )
  ament_add_pytest_test(test_serial_probe test/test_serial_probe.py
    TIMEOUT 30
  )
  ament_add_pytest_test(test_imu_attitude test/test_imu_attitude.py
    TIMEOUT 30
  )
  ament_add_pytest_test(test_imu_filter_node test/test_imu_filter_node.py
    TIMEOUT 30
  )
  ament_add_pytest_test(test_firmware_gearing test/test_firmware_gearing.py
    TIMEOUT 30
  )
  ament_add_pytest_test(test_firmware_display test/test_firmware_display.py
    TIMEOUT 60
  )
endif()

ament_package()
