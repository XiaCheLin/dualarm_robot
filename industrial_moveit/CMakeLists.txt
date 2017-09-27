cmake_minimum_required(VERSION 2.8.3)
project(industrial_moveit)

find_package(catkin REQUIRED COMPONENTS  control_msgs  roscpp rospy  sensor_msgs std_msgs std_srvs trajectory_msgs urdf moveit_msgs  moveit_core  moveit_ros_planning moveit_ros_planning_interface pluginlib cmake_modules geometric_shapes actionlib siasun_dualarm_msgs siasun_dualarm_driver )




catkin_package(
   CATKIN_DEPENDS 
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
#  INCLUDE_DIRS include
#  LIBRARIES industrial_moveit
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories( ${PROJECT_SOURCE_DIR}/include)
link_directories(${catkin_LIBRARY_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/lib)
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
include_directories(${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
catkin_package()



add_executable(industrial_moveit src/industrial_moveit.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(io_test src/IO_TEST.cpp
                    src/SetIO.cpp)
add_dependencies(io_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(io_test ${catkin_LIBRARIES})




