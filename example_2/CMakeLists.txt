cmake_minimum_required(VERSION 3.16)
project(ros2_control_demo_example_2 LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

ament_python_install_package(${PROJECT_NAME})

## COMPILE
add_library(
  ros2_control_demo_example_2
  SHARED
  hardware/diffbot_system.cpp
  hardware/include/ros2_control_demo_example_2/MiniPID.cpp
)

target_compile_features(ros2_control_demo_example_2 PUBLIC cxx_std_17)
target_include_directories(ros2_control_demo_example_2 PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/ros2_control_demo_example_2>
)
ament_target_dependencies(
  ros2_control_demo_example_2 PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

find_library(LGPIO_LIBRARY lgpio)
if(NOT LGPIO_LIBRARY)
  message(FATAL_ERROR "lgpio library not found")
endif()

target_link_libraries(ros2_control_demo_example_2
  PUBLIC
    ${LGPIO_LIBRARY} 
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface ros2_control_demo_example_2.xml)

#Install
install(FILES ros2_control_demo_example_2.xml
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY hardware/include/
  DESTINATION include/ros2_control_demo_example_2
)
install(
  DIRECTORY description 
  DESTINATION share/ros2_control_demo_example_2
)
install(
  DIRECTORY bringup
  DESTINATION share/ros2_control_demo_example_2
)
install(TARGETS ros2_control_demo_example_2
  EXPORT export_ros2_control_demo_example_2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  PROGRAMS
    imu/bno085_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(example_2_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_example_2_launch test/test_view_robot_launch.py)
  ament_add_pytest_test(run_example_2_launch test/test_diffbot_launch.py)
endif()

## EXPORTS
ament_export_targets(export_ros2_control_demo_example_2 HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
