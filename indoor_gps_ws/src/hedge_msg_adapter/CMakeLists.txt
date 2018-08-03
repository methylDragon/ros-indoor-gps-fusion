cmake_minimum_required(VERSION 2.8.3)
project(hedge_msg_adapter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  marvelmind_nav
  geometry_msgs
)

# Initialise the export variables
catkin_package(
  INCLUDE_DIRS src
  # LIBRARIES blah
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs marvelmind_nav geometry_msgs 
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# One per executable!
add_executable(hedge_msg_adapter src/hedge_msg_adapter.cpp)
target_link_libraries(hedge_msg_adapter ${catkin_LIBRARIES})

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#install(DIRECTORY param
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#install(DIRECTORY maps
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#install(DIRECTORY rviz
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
