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

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(packman_hardware_interface
  src/control_loop.cpp src/control_loop.hpp
  src/messages.cpp src/messages.hpp
  src/robot_hw.cpp src/robot_hw.hpp
  src/nmt.cpp src/nmt.hpp
  src/packman_interface.cpp src/packman_interface.hpp
)
target_link_libraries(packman_hardware_interface ${catkin_LIBRARIES})

add_executable(packman_hardware
  src/main.cpp
)
target_link_libraries(packman_hardware packman_hardware_interface)

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

set(ROSLINT_CPP_OPTS "--filter=-build/header_guard,-whitespace/braces")
ruvu_lint_add_test()
