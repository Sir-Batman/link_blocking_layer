cmake_minimum_required(VERSION 2.8.3)
project(link_blocking_layer)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES link_blocking_layer
  CATKIN_DEPENDS costmap_2d dynamic_reconfigure roscpp agent_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories( include ${catkin_INCLUDE_DIRS})

# Declare a cpp library
add_library(link_blocking_layer
  src/link_blocking_layer.cpp
)



# Specify libraries to link a library or executable target against
target_link_libraries(link_blocking_layer
  ${catkin_LIBRARIES}
)

