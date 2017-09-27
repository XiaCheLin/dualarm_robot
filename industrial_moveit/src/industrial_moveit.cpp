#include "industrial_moveit.h"
#include "siasun_dualarm_msgs/IORW.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
using namespace std;
vector<double> group_variable_values;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "industrial_moveit_node");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    moveit::planning_interface::MoveGroup group("arms");
    moveit::planning_interface::MoveGroup groupl("arm_l");
    group.setPlannerId("LBKPIECEkConfigDefault");
    groupl.setPlannerId("LBKPIECEkConfigDefault");
    moveit::planning_interface::MoveGroup::Plan my_plan;
    while (ros::ok())
    {
        vector<double> group_variable_arms;
        group_variable_values.resize(14);
        group_variable_arms.resize(14);
        moveit_msgs::RobotTrajectory trajectory;
        while(ros::ok())
        {
            group_variable_values.resize(7);
            group_variable_values[0]=-1.370492;
            group_variable_values[1]=-0.63056;
            group_variable_values[2]=-0.20755;
            group_variable_values[3]=1;
            group_variable_values[4]=0;
            group_variable_values[5]=-1.489387;
            group_variable_values[6]=1.949833;
            groupl.setJointValueTarget(group_variable_values);
            groupl.setStartStateToCurrentState();
            groupl.plan(my_plan);
            groupl.execute(my_plan);
            sleep(5);
            std::vector<geometry_msgs::Pose> waypoints;
            geometry_msgs::Pose start_pose2=groupl.getCurrentPose().pose;
            geometry_msgs::Pose target_pose3 = start_pose2;
            target_pose3.position.z -= 0.05;
            waypoints.push_back(target_pose3);  // up
            target_pose3.position.y -= 0.1;
            waypoints.push_back(target_pose3);  // left
            target_pose3.position.z -= 0.2;
            target_pose3.position.y += 0.2;
            target_pose3.position.x -= 0.2;
            waypoints.push_back(target_pose3);  // down and right
            double fra=groupl.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
            my_plan.trajectory_=trajectory;
            groupl.execute(my_plan);
            sleep(5);
            group_variable_values.resize(14);
            group_variable_values[0]=-1.370492;
            group_variable_values[1]=-0.63056;
            group_variable_values[2]=-0.20755;
            group_variable_values[3]=-0.915570;
            group_variable_values[4]=0.076207;
            group_variable_values[5]=-1.489387;
            group_variable_values[6]=1.949833;
            group_variable_values[7]=1.428040;
            group_variable_values[8]=0.616504;
            group_variable_values[9]=0.26498;
            group_variable_values[10]=1.11130;
            group_variable_values[11]=-0.1844732;
            group_variable_values[12]=1.34067;
            group_variable_values[13]=-1.8130454;
            group.setJointValueTarget(group_variable_values);
            //group.plan(my_plan);
            while(!group.move());
            sleep(5);
        }
        return 0;

    }
}

