cmake_minimum_required(VERSION 2.8.3)
project(flysky_ibus)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs
) 

###########
## Build ##
###########

include_directories(
  include
  include/serial
  ${catkin_INCLUDE_DIRS}
)

file(GLOB serial_src "include/serial/*.cc")
add_library(serial SHARED ${serial_src})

add_library(ibus SHARED include/ibus.cpp)

add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ibus
  serial
)

add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
