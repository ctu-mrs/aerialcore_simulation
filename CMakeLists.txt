cmake_minimum_required(VERSION 3.2.0)
project(aerialcore_simulation)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -Wall")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  tf2
  tf2_ros
  tf2_msgs
  geometry_msgs
  gazebo_plugins
)

# includei Gazebo
find_package(gazebo REQUIRED)

# include Qt
find_package(Qt5Widgets REQUIRED)

# include Boost
find_package(Boost REQUIRED COMPONENTS thread)

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

# XXX this approach is extremely error prone
# it would be preferable to either depend on the
# compiled headers from Gazebo directly
# or to have something entirely independent.
#
set(PROTOBUF_IMPORT_DIRS "")
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

# PROTOBUF_IMPORT_DIRS has to be set before
find_package(Protobuf REQUIRED)

set(sensor_msgs
  msgs/SITLGps.proto
  msgs/Groundtruth.proto
  )

PROTOBUF_GENERATE_CPP(SEN_PROTO_SRCS SEN_PROTO_HDRS ${sensor_msgs})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

###################################
## catkin specific configuration ##
###################################

catkin_package(
   # INCLUDE_DIRS include
   CATKIN_DEPENDS geometry_msgs tf2_msgs gazebo_plugins
   DEPENDS GAZEBO Qt5Widgets Boost Eigen
   LIBRARIES SpawnerGazeboPlugin 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${catkin_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  )

link_directories(${GAZEBO_LIBRARY_DIRS})

##  world plugins
add_library(SpawnerGazeboPlugin SHARED src/world_plugins/spawner_gazebo_plugin.cc)
add_library(PowerlineGazeboPlugin SHARED src/world_plugins/powerline_gazebo_plugin.cc)

add_library(DynamicModelPlugin SHARED src/world_plugins/dynamic_model_plugin.cpp)
target_link_libraries(SpawnerGazeboPlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  )
