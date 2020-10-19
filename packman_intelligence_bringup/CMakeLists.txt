cmake_minimum_required(VERSION 2.8.3)
project(packman_intelligence_bringup)

find_package(catkin REQUIRED COMPONENTS
  ruvu_lint
)

###################################
## catkin specific configuration ##
###################################

catkin_package()

#############
## Install ##
#############

install(DIRECTORY
  config
  launch
  maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

ruvu_lint_add_test()

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
endif()
