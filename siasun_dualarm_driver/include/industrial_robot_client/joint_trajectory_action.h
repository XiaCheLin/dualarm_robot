/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef JOINT_TRAJTORY_ACTION_H
#define JOINT_TRAJTORY_ACTION_H

#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include "industrial_robot_client/siasun_dualarm_utils.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <siasun_dualarm_msgs/RobotStatus.h>
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
namespace industrial_robot_client
{
namespace joint_trajectory_action
{

class JointTrajectoryAction
{

public:

    /**
   * \brief Constructor
   *
   */
    JointTrajectoryAction();

    /**
   * \brief Destructor
   *
   */
    ~JointTrajectoryAction();

    /**
     * \brief Begin processing messages and publishing topics.
     */
    void run() { ros::spin(); }

    static void setRobotIp(int robot_ip);

    static int getRobotIp();

private:


    static const double WATCHD0G_PERIOD_;// = 1.0;

    static int robot_ip_;


    typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> JointTractoryActionServer;

    /**
   * \brief Internal ROS node handle
   */
    ros::NodeHandle node_;


    actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>* actionServer_;
    std::map<int, JointTractoryActionServer*> act_servers_;
    /**
   * \brief Internal action server
   */
    JointTractoryActionServer action_server_;

    /**
   * \brief Publishes desired trajectory (typically to the robot driver)
   */
    ros::Publisher pub_trajectory_command_;
    std::map<int, ros::Publisher> pub_trajectories_;


    /**
   * \brief Subscribes to trajectory feedback (typically published by the
   * robot driver).
   */
    ros::Subscriber sub_trajectory_state_;
    std::map<int, ros::Subscriber> sub_trajectories_;

    /**
   * \brief Subscribes to the robot status (typically published by the
   * robot driver).
   */
    ros::Subscriber sub_robot_status_;
    std::map<int, ros::Subscriber> sub_status_;

    ros::Subscriber sub_robot_collision_;

    std::vector<std::string> all_joint_names_;

    /**
   * \brief Watchdog time used to fail the action request if the robot
   * driver is not responding.
   */
    ros::Timer watchdog_timer_;
    std::map<int, ros::Timer>watchdog_timer_map_;
    /**
    * \brief Controller was alive during the last watchdog interval
    */
    bool controller_alive_;

    std::map<int, bool> controller_alive_map_;
    /**
   * \brief Indicates action has an active goal
   */
    bool has_active_goal_;

    std::map<int, bool> has_active_goal_map_;

    /**
   * \brief Indicates that the robot has been in a moving state at least once since
   * starting the current active trajectory
   */
    bool has_moved_once_;

    std::map<int, bool> has_moved_once_map_;
    /**
   * \brief Cache of the current active goal
   */
    JointTractoryActionServer::GoalHandle active_goal_;

    std::map<int, JointTractoryActionServer::GoalHandle> active_goal_map_;
    /**
   * \brief Cache of the current active trajectory
   */
    trajectory_msgs::JointTrajectory current_traj_;

    std::map<int, trajectory_msgs::JointTrajectory> current_traj_map_;
    /**
   * \brief The default goal joint threshold see(goal_threshold). Unit
   * are joint specific (i.e. radians or meters).
   */
    static const double DEFAULT_GOAL_THRESHOLD_;// = 0.01;

    /**
   * \brief The goal joint threshold used for determining if a robot
   * is near it final destination.  A single value is used for all joints
   *
   * NOTE: This value is used in conjunction with the robot inMotion
   * status (see siasun_dualarm_msgs::RobotStatus) if it exists.
   */
    double goal_threshold_;

    /**
   * \brief The joint names associated with the robot the action is
   * interfacing with.  The joint names must be the same as expected
   * by the robot driver.
   */
    std::vector<std::string> joint_names_;

    /**
   * \brief Cache of the last subscribed feedback message
   */
    control_msgs::FollowJointTrajectoryFeedbackConstPtr last_trajectory_state_;
    std::map<int, control_msgs::FollowJointTrajectoryFeedbackConstPtr> last_trajectory_state_map_;

    /**
   * \brief Cache of the last subscribed status message
   */
    siasun_dualarm_msgs::RobotStatusConstPtr last_robot_status_;

    /**
   * \brief Time at which to start checking for completion of current
   * goal, if one is active
   */
    ros::Time time_to_check_;
    std::map<int,ros::Time>time_to_check_map_;

    /**
   * \brief The watchdog period (seconds)
   */
    static  double WATCHDOG_PERIOD_;// = 1.0;

    /**
   * \brief Watch dog callback, used to detect robot driver failures
   *
   * \param e time event information
   *
   */
    void watchdog(const ros::TimerEvent &e);

    void watchdog(const ros::TimerEvent &e, int group_number);

    /**
   * \brief Action server goal callback method
   *
   * \param gh goal handle
   *
   */
    void goalCB(JointTractoryActionServer::GoalHandle gh);

    void goalExCB(JointTractoryActionServer::GoalHandle gh, int group_number);
    /**
   * \brief Action server cancel callback method
   *
   * \param gh goal handle
   *
   */

    void cancelCB(JointTractoryActionServer::GoalHandle gh);

    void cancelExCB(JointTractoryActionServer::GoalHandle gh, int group_number);
    /**
   * \brief Controller state callback (executed when feedback message
   * received)
   *
   * \param msg joint trajectory feedback message
   *
   */
    void controllerStateCB(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg);

    void controllerStateCB(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg, int robot_id);
    /**
   * \brief Controller status callback (executed when robot status
   *  message received)
   *
   * \param msg robot status message
   *
   */
    void robotStatusCB(const siasun_dualarm_msgs::RobotStatusConstPtr &msg);

    void robotCollision(const std_msgs::String::ConstPtr &msg);

    /**
   * \brief Aborts the current action goal and sends a stop command
   * (empty message) to the robot driver.
   *
   *
   */
    void abortGoal();

    void abortGoal(int robot_id);
    /**
   * \brief Controller status callback (executed when robot status
   *  message received)
   *
   * \param msg trajectory feedback message
   * \param traj trajectory to test against feedback
   *
   * \return true if all joints are within goal contraints
   *
   */
    bool withinGoalConstraints(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg,const trajectory_msgs::JointTrajectory & traj);

    bool withinGoalConstraints(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg,const trajectory_msgs::JointTrajectory & traj, int robot_id);

    std::map<int, RobotGroup> robot_groups_;

    ros::Publisher robotIp_;

    std_msgs::Int32 msg_;

    std::vector<bool> bHasSync_;
};

} //joint_trajectory_action
} //industrial_robot_client

#endif /* JOINT_TRAJTORY_ACTION_H */
