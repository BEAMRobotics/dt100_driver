cmake_minimum_required(VERSION 3.14)
project(dt100_driver)
set(CMAKE_CXX_STANDARD 14)

find_package(Boost REQUIRED)
find_package(PCL  REQUIRED)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    pcl_ros
    pcl_conversions
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    std_msgs
    sensor_msgs
    roscpp
  DEPENDS
    BOOST
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Build ##
# Relay library
add_library(${PROJECT_NAME}
  src/dt100_relay_client.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Relay node
add_executable(dt100_relay src/dt100_relay.cpp)

target_link_libraries(dt100_relay
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#Virtual Machine node
add_executable(dt100_virtual_machine src/dt100_virtual_machine.cpp)

target_link_libraries(dt100_virtual_machine
   ${catkin_LIBRARIES}
)
