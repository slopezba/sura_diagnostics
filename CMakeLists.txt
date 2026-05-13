cmake_minimum_required(VERSION 3.8)
project(sura_diagnostics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sura_msgs REQUIRED)

include_directories(include)

set(COMMON_DEPENDENCIES
  controller_manager_msgs
  diagnostic_msgs
  geometry_msgs
  lifecycle_msgs
  rcl_interfaces
  rclcpp
  sensor_msgs
  std_srvs
  sura_msgs
)

set(DIAGNOSTIC_EXECUTABLES
  battery_status_diagnostics
  controller_debug_diagnostics
  dvl_altitude_diagnostics
  dvl_diagnostics
  gps_diagnostics
  hardware_components_diagnostics
  imu_diagnostics
  leak_sensors_diagnostics
  navigation_limits_diagnostics
  pressure_diagnostics
  robot_watchdog
)

foreach(target ${DIAGNOSTIC_EXECUTABLES})
  add_executable(${target} src/${target}.cpp)
  target_compile_features(${target} PUBLIC cxx_std_17)
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${target} ${COMMON_DEPENDENCIES})
endforeach()

install(TARGETS ${DIAGNOSTIC_EXECUTABLES}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
