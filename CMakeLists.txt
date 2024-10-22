cmake_minimum_required(VERSION 3.8)
project(hoverboard_control)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

# Uncomment and add other dependencies if needed
# find_package(<dependency> REQUIRED)

# Automatically find build dependencies
ament_auto_find_build_dependencies()

# Add serial_port_service as a shared library
ament_auto_add_library(
  serial_port_service SHARED
  src/serial_port_service.cpp
)

# Link Boost libraries to serial_port_service
target_link_libraries(serial_port_service PRIVATE Boost::system Boost::thread)

# Add hoverboard_hardware_interface as a shared library
ament_auto_add_library(
  hoverboard_hardware_interface SHARED
  src/hoverboard_hardware_interface.cpp
)

# Link serial_port_service and Boost libraries to hoverboard_hardware_interface
target_link_libraries(hoverboard_hardware_interface PRIVATE serial_port_service Boost::system Boost::thread)

# Ensure that the hoverboard_hardware_interface links to required packages
ament_target_dependencies(hoverboard_hardware_interface
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  Boost
)

# Export plugin description file
pluginlib_export_plugin_description_file(hardware_interface hoverboard_hardware_interface_plugins.xml)

# Install libraries
install(TARGETS
  serial_port_service
  hoverboard_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Install plugin description file
install(FILES hoverboard_hardware_interface_plugins.xml
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install URDF files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Handle testing dependencies and linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The following line skips the linter which checks for copyrights
  # Comment the line when a copyright and license is added
  set(ament_cmake_copyright_FOUND TRUE)
  # The following line skips cpplint (only works in a git repo)
  # Comment the line when this package is in a git repo and when
  # a copyright and license are added
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Finalize the package with ament_auto_package
ament_auto_package()
