/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013-2015, TU Delft Robotics Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the TU Delft Robotics Institute nor the names
 *    of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * Author: G.A. vd. Hoorn - TU Delft Robotics Institute
 */

#include <industrial_robot_client/joint_relay_handler.h>
#include <industrial_robot_client/joint_trajectory_downloader.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <vector>
#include <stdexcept>
#include <ros/ros.h>
#include <urdf/model.h>
#include <industrial_utils/param_utils.h>
#include "industrial_robot_client/joint_trajectory_streamer.h"
#include "industrial_robot_client/robot_state_interface.h"
#include <string>
using industrial_robot_client::joint_trajectory_downloader::JointTrajectoryDownloader;
using namespace std;


int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc,argv,"motion_interface");

  // launch the default JointTrajectoryStreamer connection/handlers
 JointTrajectoryDownloader motionInterface;
  motionInterface.init();
  motionInterface.run();

  return 0;
}
