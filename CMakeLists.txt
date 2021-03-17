project(DT100_driver)
cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)

find_package(Boost REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(beam REQUIRED utils)

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
  src/DT100RelayClient.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
)
target_link_libraries(${PROJECT_NAME}
  beam::beam
  ${catkin_LIBRARIES}
)

# Relay node
add_executable(DT100_relay src/DT100_relay.cpp
)
target_link_libraries(DT100_relay
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

#Virtual Machine node
add_executable(DT100_virtual_machine src/DT100_virtual_machine.cpp)

target_link_libraries(DT100_virtual_machine
   ${catkin_LIBRARIES}
)
