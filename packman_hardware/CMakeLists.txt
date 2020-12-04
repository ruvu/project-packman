cmake_minimum_required(VERSION 3.5)
project(packman_hardware LANGUAGES CXX)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(Boost REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(socketcan_interface REQUIRED)

add_library(${PROJECT_NAME}
  src/messages.cpp src/messages.hpp
  src/nmt.cpp src/nmt.hpp
  src/packman_interface.cpp src/packman_interface.hpp
  src/system_interface.cpp src/system_interface.hpp
)
ament_target_dependencies(
  ${PROJECT_NAME}
  Boost
  controller_manager
  hardware_interface
  socketcan_interface
)

pluginlib_export_plugin_description_file(hardware_interface plugins_description.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
