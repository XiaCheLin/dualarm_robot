#ifndef ArmRobot_H
#define ArmRobot_H
//Socket include
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/time.h>
#include <linux/input.h>
#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <errno.h>
#include<assert.h>     

// ROS includes
#include <ros/ros.h>
#include <urdf/model.h>
#include<unistd.h>
#include<sys/ipc.h>
// ROS message includes
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <moveit_msgs/DisplayTrajectory.h>
 
//ROS moveit
#include <pluginlib/class_loader.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/PlanningScene.h>
#include "moveit/trajectory_processing/iterative_time_parameterization.h"
//math.h
#include <math.h>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/tokenizer.hpp>
#include <vector>
//mongod
#include "moveit/warehouse/warehouse_connector.h"
#include "moveit/warehouse/moveit_message_storage.h"
#include <sys/sem.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include<semaphore.h>
#include<pthread.h>
#include <moveit/warehouse/state_storage.h>
#define BOOST_DATE_TIME_SOURCE



#endif
