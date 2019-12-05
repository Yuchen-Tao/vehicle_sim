cmake_minimum_required(VERSION 2.8.3)
project(vehicle_sim)

## Compile as C++11, supported in ROS Kinetic and newer
# 
#add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  automated_driving_msgs
)
find_package(SDL REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES
 CATKIN_DEPENDS automated_driving_msgs
 DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(vehicle_sim_lib 
src/vehicle_sim.cpp
)

add_executable(vehicle_sim_node src/vehicle_sim_node.cpp)

add_dependencies(vehicle_sim_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(vehicle_sim_node vehicle_sim_lib
  ${catkin_LIBRARIES}
  ${SDL_LIBRARY}
)
