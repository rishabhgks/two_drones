cmake_minimum_required(VERSION 2.8.3)
project(two_drones)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# set this to path to AirSim root folder if you want your catkin workspace in a custom directory

set(CMAKE_CXX_STANDARD 11)
set(CXX_EXP_LIB "-nostdinc++ -I/usr/include/c++/8 -I/usr/include/x86_64-linux-gnu/c++/8 -nodefaultlibs 
-l/usr/lib/x86_64-linux-gnu/libc++.so -l/usr/lib/x86_64-linux-gnu/libc++abi.so
-lm -lc -lgcc_s -lgcc  
-lstdc++fs -fmax-errors=10 -Wnoexcept -Wstrict-null-sentinel") 

set(OpenCV_INCLUDE_DIRS "/usr/include/opencv4;/usr/include")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        geometry_msgs
        dji_sdk
        image_transport # for message_filter
        )

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES
 CATKIN_DEPENDS roscpp rospy std_msgs urdf
 DEPENDS
)

find_package(DJIOSDK REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(hover_node src/hover_node.cpp)

add_dependencies(hover_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(hover_node
  ${catkin_LIBRARIES}
  ${DJIOSDK_LIBRARIES}
)

add_executable(four_drone_node src/four_drone_node.cpp)

add_dependencies(four_drone_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(four_drone_node
  ${catkin_LIBRARIES}
  ${DJIOSDK_LIBRARIES}
)

add_executable(odom_pub_node src/odom_pub_node.cpp)

add_dependencies(odom_pub_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(odom_pub_node
  ${catkin_LIBRARIES}
)

# add_executable(test_node src/test_node.cpp)

# target_link_libraries(test_node
#   ${catkin_LIBRARIES}
# )
# add_dependencies(test_node ${catkin_EXPORTED_TARGETS})

add_executable(takeoff_node src/takeoff_node.cpp)

target_link_libraries(takeoff_node
  ${catkin_LIBRARIES}
  ${DJIOSDK_LIBRARIES}
)
add_dependencies(takeoff_node ${catkin_EXPORTED_TARGETS})

add_executable(beam_forming_node src/beam_forming_node.cpp)

target_link_libraries(beam_forming_node
  ${catkin_LIBRARIES}
  ${DJIOSDK_LIBRARIES}
)
add_dependencies(beam_forming_node ${catkin_EXPORTED_TARGETS})

add_executable(one_uav_diamond_node src/one_uav_diamond_node.cpp)

target_link_libraries(one_uav_diamond_node
  ${catkin_LIBRARIES}
  ${DJIOSDK_LIBRARIES}
)
add_dependencies(one_uav_diamond_node ${catkin_EXPORTED_TARGETS})

add_executable(two_uav_diamond_node src/two_uav_diamond_node.cpp)

target_link_libraries(two_uav_diamond_node
  ${catkin_LIBRARIES}
  ${DJIOSDK_LIBRARIES}
)
add_dependencies(two_uav_diamond_node ${catkin_EXPORTED_TARGETS})

add_executable(one_uav_dia_opti_node src/one_uav_dia_opti_node.cpp)

target_link_libraries(one_uav_dia_opti_node
  ${catkin_LIBRARIES}
  ${DJIOSDK_LIBRARIES}
)
add_dependencies(one_uav_dia_opti_node ${catkin_EXPORTED_TARGETS})

add_executable(two_uav_dia_opti_node src/two_uav_dia_opti_node.cpp)

target_link_libraries(two_uav_dia_opti_node
  ${catkin_LIBRARIES}
  ${DJIOSDK_LIBRARIES}
)
add_dependencies(two_uav_dia_opti_node ${catkin_EXPORTED_TARGETS})

add_executable(three_uav_dia_opti_node src/three_uav_dia_opti_node.cpp)

target_link_libraries(three_uav_dia_opti_node
  ${catkin_LIBRARIES}
  ${DJIOSDK_LIBRARIES}
)
add_dependencies(three_uav_dia_opti_node ${catkin_EXPORTED_TARGETS})

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
