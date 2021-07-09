cmake_minimum_required(VERSION 3.0.2)
project(packman_desktop_bringup)

find_package(catkin REQUIRED COMPONENTS
  ruvu_lint
)

catkin_add_env_hooks(99.packman_desktop_bringup_functions SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

###################################
## catkin specific configuration ##
###################################

catkin_package()

#############
## Install ##
#############

install(DIRECTORY
  config
  icons
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

ruvu_lint_add_test()

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
