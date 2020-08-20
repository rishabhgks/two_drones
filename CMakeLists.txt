cmake_minimum_required(VERSION 2.8.3)
project(two_drones)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# set this to path to AirSim root folder if you want your catkin workspace in a custom directory
set(AIRSIM_ROOT /home/rishab/Downloads/Github/AirSim/)

add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)
add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" MavLinkCom)

set(CMAKE_CXX_STANDARD 11)
set(CXX_EXP_LIB "-nostdinc++ -I/usr/include/c++/8 -I/usr/include/x86_64-linux-gnu/c++/8 -nodefaultlibs 
-l/usr/lib/x86_64-linux-gnu/libc++.so -l/usr/lib/x86_64-linux-gnu/libc++abi.so
-lm -lc -lgcc_s -lgcc  
-lstdc++fs -fmax-errors=10 -Wnoexcept -Wstrict-null-sentinel") 

set(OpenCV_INCLUDE_DIRS "/usr/include/opencv4;/usr/include")

set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/rpclib-2.2.1/include")
set(RPC_LIB rpc) # name of .a file with lib prefix

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  urdf
  rospy
  std_msgs
  geometry_msgs
  geographic_msgs
  geodesy
  airsim_ros_pkgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES
 CATKIN_DEPENDS roscpp rospy std_msgs urdf airsim_ros_pkgs
 DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${AIRSIM_ROOT}/AirLib/deps/eigen3
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
)

add_executable(hover_node src/hover_node.cpp)

add_dependencies(hover_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(hover_node
  ${catkin_LIBRARIES}
)

add_executable(test_node src/test_node.cpp)

target_link_libraries(test_node
  ${catkin_LIBRARIES}
)
add_dependencies(test_node ${catkin_EXPORTED_TARGETS})

# add_executable(takeoff_node src/takeoff_node.cpp)

# target_link_libraries(takeoff_node
#   ${catkin_LIBRARIES}
# )
# add_dependencies(takeoff_node ${catkin_EXPORTED_TARGETS})

# add_executable(obstacle_avoidance src/obstacle_avoidance.cpp)

# target_link_libraries(obstacle_avoidance
#   ${catkin_LIBRARIES}
# )
# add_dependencies(obstacle_avoidance ${catkin_EXPORTED_TARGETS})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
