cmake_minimum_required(VERSION 3.8)
project(lift_slide_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(openflex_can REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(lift_slide_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/lift_slide_hardware_interface.cpp
  src/param_forwarding_controller_base.cpp
  src/param_forwarding_velocity_controller.cpp
  src/param_forwarding_position_controller.cpp
  src/lift_slide_state_controller.cpp
  src/lift_slide_manual_position_controller.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  hardware_interface
  openflex_can
  lift_slide_msgs
  pluginlib
  rclcpp
  rcl_interfaces
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  std_srvs
)

target_link_libraries(${PROJECT_NAME}
  openflex_can::openflex_can
)

pluginlib_export_plugin_description_file(hardware_interface lift_slide_driver.xml)
pluginlib_export_plugin_description_file(controller_interface lift_slide_driver_controllers.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_lift_contract
    test/test_lift_contract.py
  )
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  controller_interface
  hardware_interface
  openflex_can
  lift_slide_msgs
  pluginlib
  rclcpp
  rcl_interfaces
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  std_srvs
)

ament_package()
