cmake_minimum_required(VERSION 2.8.3)
project(siasun_dualarm_driver)

find_package(catkin REQUIRED COMPONENTS control_msgs diagnostic_msgs roscpp  sensor_msgs std_msgs std_srvs trajectory_msgs urdf rospy actionlib trajectory_msgs actionlib_msgs  moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
 siasun_dualarm_msgs roslint industrial_utils)
find_package(Boost REQUIRED system filesystem date_time thread)
set(ROS_BUILD_STATIC_LIBS true)
set(ROS_BUILD_SHARED_LIBS false)
roslint_cpp()
add_definitions(-DROS=1)
add_definitions(-DLINUXSOCKETS=1)
include_directories(${PROJECT_SOURCE_DIR}/lib)
include_directories(${PROJECT_SOURCE_DIR}/src/industrial_robot_client)
include_directories(${PROJECT_SOURCE_DIR}/src/simple_message)
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories()

catkin_package(
    DEPENDS
    CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    std_msgs roscpp actionlib trajectory_msgs actionlib_msgs
    control_msgs sensor_msgs  urdf industrial_utils
    INCLUDE_DIRS include
    LIBRARIES
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}_dummy
)



#simple_message


set(    SRC_FILES
        src/simple_message/byte_array.cpp
        src/simple_message/simple_message.cpp
        src/simple_message/smpl_msg_connection.cpp
        src/simple_message/socket/simple_socket.cpp
        src/simple_message/socket/udp_socket.cpp
        src/simple_message/socket/udp_client.cpp
        src/simple_message/socket/udp_server.cpp
        src/simple_message/socket/tcp_socket.cpp
        src/simple_message/socket/tcp_client.cpp
        src/simple_message/socket/tcp_server.cpp
        src/simple_message/message_handler.cpp
        src/simple_message/message_manager.cpp
        src/simple_message/ping_handler.cpp
        src/simple_message/ping_message.cpp
        src/simple_message/joint_data.cpp
        src/simple_message/joint_feedback.cpp
        src/simple_message/joint_traj_pt.cpp
        src/simple_message/joint_traj_pt_full.cpp
        src/simple_message/joint_traj.cpp
        src/simple_message/robot_status.cpp
        src/simple_message/messages/joint_message.cpp
        src/simple_message/messages/joint_feedback_message.cpp
        src/simple_message/messages/joint_traj_pt_message.cpp
        src/simple_message/messages/joint_traj_pt_full_message.cpp
        src/simple_message/messages/robot_status_message.cpp
        src/simple_message/simple_comms_fault_handler.cpp)

set(CLIENT_SRC_FILES
  src/industrial_robot_client/joint_relay_handler.cpp
  src/industrial_robot_client/robot_state_interface.cpp
  src/industrial_robot_client/joint_trajectory_interface.cpp
  src/industrial_robot_client/joint_trajectory_streamer.cpp
  src/industrial_robot_client/joint_trajectory_downloader.cpp
  src/industrial_robot_client/robot_status_relay_handler.cpp
  src/industrial_robot_client/siasun_dualarm_utils.cpp
  src/industrial_robot_client/robot_group.cpp
  src/simple_message/messages/joint_traj_pt_message.cpp
  src/simple_message/robot_status.cpp
  src/industrial_robot_client/utils.cpp
  src/industrial_robot_client/IO_states_config.cpp
  src/industrial_robot_client/system_safty.cpp)

add_custom_command(
  OUTPUT  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_dummy.cpp
  COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_dummy.cpp)
add_library(${PROJECT_NAME}_dummy ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_dummy.cpp)

install(TARGETS ${PROJECT_NAME}_dummy DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

add_library(simple_message ${SRC_FILES})
target_link_libraries(simple_message ${catkin_LIBRARIES})
add_dependencies(simple_message ${siasun_dualarm_msgs_EXPORTED_TARGETS})

add_library(simple_message_bswap ${SRC_FILES})
set_target_properties(simple_message_bswap PROPERTIES COMPILE_DEFINITIONS "BYTE_SWAPPING")
target_link_libraries(simple_message_bswap ${catkin_LIBRARIES})
add_dependencies(simple_message_bswap ${siasun_dualarm_msgs_EXPORTED_TARGETS})


add_library(simple_message_float64 ${SRC_FILES})
set_target_properties(simple_message_float64 PROPERTIES COMPILE_DEFINITIONS "FLOAT64")
target_link_libraries(simple_message_float64 ${catkin_LIBRARIES})
add_dependencies(simple_message_float64 ${siasun_dualarm_msgs_EXPORTED_TARGETS})



install(
    TARGETS simple_message simple_message_bswap simple_message_float64
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


add_library(siasun_dualarm_industrial_robot_client ${CLIENT_SRC_FILES})
target_link_libraries(siasun_dualarm_industrial_robot_client
  simple_message)
add_dependencies(siasun_dualarm_industrial_robot_client ${catkin_EXPORTED_TARGETS})

add_library(siasun_dualarm_industrial_robot_client_bswap ${CLIENT_SRC_FILES})
set_target_properties(siasun_dualarm_industrial_robot_client PROPERTIES COMPILE_DEFINITIONS "BYTE_SWAPPING")
target_link_libraries(siasun_dualarm_industrial_robot_client_bswap)
add_dependencies(siasun_dualarm_industrial_robot_client_bswap ${catkin_EXPORTED_TARGETS})

add_executable(siasun_dualarm_joint_trajectory_action
  src/joint_trajectory_node.cpp
  src/industrial_robot_client/joint_trajectory_action.cpp
  src/industrial_robot_client/siasun_dualarm_utils.cpp)
target_link_libraries(siasun_dualarm_joint_trajectory_action
  simple_message
  siasun_dualarm_industrial_robot_client
  ${catkin_LIBRARIES})
set_target_properties(siasun_dualarm_joint_trajectory_action
  PROPERTIES OUTPUT_NAME joint_trajectory_action
  PREFIX "")

add_executable(siasun_dualarm_robot_state
  src/robot_state_node.cpp
  src/industrial_robot_client/robot_state_interface.cpp
  src/industrial_robot_client/robot_status_relay_handler.cpp
  src/industrial_robot_client/joint_relay_handler.cpp
  src/simple_message/robot_status.cpp)

target_link_libraries(siasun_dualarm_robot_state
  simple_message
  siasun_dualarm_industrial_robot_client
  ${catkin_LIBRARIES})
set_target_properties(siasun_dualarm_robot_state
  PROPERTIES OUTPUT_NAME siasun_robot_state
  PREFIX "")


add_executable(siasun_dualarm_robot_state_bswap
  src/robot_state_node.cpp
  src/industrial_robot_client/robot_state_interface.cpp
  src/industrial_robot_client/robot_status_relay_handler.cpp
  src/industrial_robot_client/joint_relay_handler.cpp
  src/simple_message/robot_status.cpp)
target_link_libraries(siasun_dualarm_robot_state_bswap
  simple_message_bswap
  siasun_dualarm_industrial_robot_client_bswap
  ${catkin_LIBRARIES})
set_target_properties(siasun_dualarm_robot_state_bswap
  PROPERTIES OUTPUT_NAME siasun_robot_state_bswap
  PREFIX "")


add_executable(siasun_dualarm_motion_download_interface
  src/joint_download_node.cpp
  src/industrial_robot_client/joint_trajectory_downloader.cpp
  src/industrial_robot_client/siasun_dualarm_utils.cpp
  src/industrial_robot_client/joint_trajectory_interface.cpp
  src/simple_message/messages/joint_traj_pt_message.cpp)

target_link_libraries(siasun_dualarm_motion_download_interface
  simple_message
  siasun_dualarm_industrial_robot_client
  ${catkin_LIBRARIES})
set_target_properties(siasun_dualarm_motion_download_interface
  PROPERTIES OUTPUT_NAME motion_download_interface
  PREFIX "")

add_executable(siasun_dualarm_motion_download_interface_bswap
  src/joint_download_node.cpp
  src/industrial_robot_client/joint_trajectory_downloader.cpp
  src/industrial_robot_client/siasun_dualarm_utils.cpp
  src/industrial_robot_client/joint_trajectory_interface.cpp
  src/simple_message/messages/joint_traj_pt_message.cpp)

target_link_libraries(siasun_dualarm_motion_download_interface_bswap
  simple_message_bswap
  siasun_dualarm_industrial_robot_client_bswap
  ${catkin_LIBRARIES})
set_target_properties(siasun_dualarm_motion_download_interface_bswap
  PROPERTIES OUTPUT_NAME motion_download_interface_bswap
  PREFIX "")

add_executable(siasun_dualarm_motion_streaming_interface
  src/joint_streamer_node.cpp
  src/industrial_robot_client/joint_trajectory_streamer.cpp
  src/industrial_robot_client/siasun_dualarm_utils.cpp
  src/industrial_robot_client/joint_trajectory_interface.cpp
  src/simple_message/messages/joint_traj_pt_message.cpp)

target_link_libraries(siasun_dualarm_motion_streaming_interface
  simple_message
  siasun_dualarm_industrial_robot_client
  ${catkin_LIBRARIES})
set_target_properties(siasun_dualarm_motion_streaming_interface
  PROPERTIES OUTPUT_NAME motion_streaming_interface
  PREFIX "")

add_executable(siasun_dualarm_motion_streaming_interface_bswap
  src/joint_streamer_node.cpp
  src/industrial_robot_client/joint_trajectory_streamer.cpp
  src/industrial_robot_client/siasun_dualarm_utils.cpp
  src/industrial_robot_client/joint_trajectory_interface.cpp
  src/simple_message/messages/joint_traj_pt_message.cpp)

target_link_libraries(siasun_dualarm_motion_streaming_interface_bswap
  simple_message_bswap
  siasun_dualarm_industrial_robot_client_bswap
  ${catkin_LIBRARIES})

set_target_properties(siasun_dualarm_motion_streaming_interface_bswap
  PROPERTIES OUTPUT_NAME motion_streaming_interface_bswap
  PREFIX "")

add_executable(siasun_dualarm_IO_states
  src/industrial_robot_client/IO_states_config.cpp
  src/IO_states_node)
target_link_libraries(siasun_dualarm_IO_states
  simple_message
  siasun_dualarm_industrial_robot_client
  ${catkin_LIBRARIES})
set_target_properties(siasun_dualarm_IO_states
  PROPERTIES OUTPUT_NAME IO_states
  PREFIX "")

add_executable(siasun_dualarm_IO_states_bswap
  src/industrial_robot_client/IO_states_config.cpp
  src/IO_states_node)
target_link_libraries(siasun_dualarm_IO_states_bswap
  simple_message_bswap
  siasun_dualarm_industrial_robot_client_bswap
  ${catkin_LIBRARIES})
set_target_properties(siasun_dualarm_IO_states_bswap
  PROPERTIES OUTPUT_NAME IO_states_bswap
  PREFIX "")

add_executable(siasun_dualarm_system_safty
  src/industrial_robot_client/system_safty.cpp
  src/system_safty_node.cpp)
target_link_libraries(siasun_dualarm_system_safty
  simple_message
  siasun_dualarm_industrial_robot_client
  ${catkin_LIBRARIES})
set_target_properties(siasun_dualarm_system_safty
  PROPERTIES OUTPUT_NAME system_safty
  PREFIX "")

add_executable(siasun_dualarm_system_safty_bswap
  src/industrial_robot_client/system_safty.cpp
  src/system_safty_node)
target_link_libraries(siasun_dualarm_system_safty_bswap
  simple_message_bswap
  siasun_dualarm_industrial_robot_client_bswap
  ${catkin_LIBRARIES})
set_target_properties(siasun_dualarm_system_safty_bswap
  PROPERTIES OUTPUT_NAME system_safty_bswap
  PREFIX "")

