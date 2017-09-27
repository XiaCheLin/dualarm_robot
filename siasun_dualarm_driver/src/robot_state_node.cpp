
#include <trajectory_msgs/JointTrajectory.h>
#include <vector>
#include <industrial_utils/param_utils.h>
#include <ros/ros.h>
#include <urdf/model.h>
#include <stdexcept>
#include "industrial_robot_client/joint_trajectory_streamer.h"
#include "industrial_robot_client/robot_state_interface.h"
#include <string>
using namespace std;
using industrial_robot_client::robot_state_interface::RobotStateInterface;
using industrial_robot_client::joint_relay_handler::JointRelayHandler;
using industrial_utils::param::getJointNames;

int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "state_interface");
  // launch the default Robot State Interface connection/handlers
  RobotStateInterface rsi;
  if(rsi.init())
  {
  // run the node
  //ros::spin();
  rsi.run();
  }
  return 0;
}
