cmake_minimum_required(VERSION 3.8)
project(hoverboard_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Boost REQUIRED COMPONENTS system) # thread regex
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(
  serial_port_service SHARED
  src/serial_port_service.cpp
)

ament_auto_add_library(
  hoverboard_hardware_interface SHARED
  src/hoverboard_hardware_interface.cpp
)

target_link_libraries(hoverboard_hardware_interface ${Boost_LIBRARIES})

pluginlib_export_plugin_description_file(hardware_interface hoverboard_hardware_interface.xml)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install URDF files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# ament_package()
ament_auto_package()