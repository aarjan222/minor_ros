cmake_minimum_required(VERSION 3.8)
project(car_gazebo LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  mpu6050driver
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  car_gazebo
  SHARED
  hardware/carlikebot_system.cpp
  hardware/crc8.cpp
)
target_compile_features(car_gazebo PUBLIC cxx_std_17)
target_include_directories(car_gazebo PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/car_gazebo>
)
# find_library(WIRINGPI_LIBRARY libwiringPi.so.2)
ament_target_dependencies(
  car_gazebo PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# target_compile_options(car_gazebo PRIVATE " -lwiringPi -lwiringPiDev ")

target_link_libraries(
    car_gazebo PRIVATE
    wiringPi
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "CAR_GAZEBO_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface car_gazebo.xml)

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
# find_package(ament_cmake_pytest REQUIRED)
# the following line skips the linter which checks for copyrights
# comment the line when a copyright and license is added to all source files
set(ament_cmake_copyright_FOUND TRUE)
# the following line skips cpplint (only works in a git repo)
# comment the line when this package is in a git repo and when
# a copyright and license is added to all source files
set(ament_cmake_cpplint_FOUND TRUE)
ament_lint_auto_find_test_dependencies()
endif()

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/car_gazebo
)

install(TARGETS car_gazebo
  EXPORT export_car_gazebo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch config transforms src urdf cv
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  transforms/map_to_base_link.py
  transforms/odom_to_base_link.py
  src/teleop.py
  cv/yolov8_ros2_pt.py
  cv/yolov8_ros2_subscriber.py
  DESTINATION lib/${PROJECT_NAME})

## EXPORTS
ament_export_targets(export_car_gazebo HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()