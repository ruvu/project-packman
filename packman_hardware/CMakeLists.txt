cmake_minimum_required(VERSION 2.8.3)
project(packman_hardware)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  roscpp
  socketcan_interface
  ruvu_lint
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(packman_hardware_interface
  src/control_loop.cpp src/control_loop.h
  src/messages.cpp src/messages.h
  src/packman_state.h
  src/robot_hw.cpp src/robot_hw.h
  src/nmt.cpp src/nmt.hpp
  src/pacman_interface.cpp src/pacman_interface.h
)
target_link_libraries(packman_hardware_interface ${catkin_LIBRARIES} -latomic)
add_dependencies(packman_hardware_interface ${catkin_EXPORTED_TARGETS})

add_executable(packman_hardware
  src/main.cpp
)
target_link_libraries(packman_hardware packman_hardware_interface)

add_executable(test_slow_speed
  src/test_slow_speed.cpp
)
target_link_libraries(test_slow_speed packman_hardware_interface)

#############
## Install ##
#############

install(TARGETS packman_hardware packman_hardware_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

ruvu_lint_add_test()
