cmake_minimum_required(VERSION 3.0.2)
project(packman_description)

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
  urdf_xacro
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

ruvu_lint_add_test()

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(test/load_urdf.test)
  roslaunch_add_file_check(test/load_urdf_with_prefix.test)
endif()
