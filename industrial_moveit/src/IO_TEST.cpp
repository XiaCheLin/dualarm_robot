#include <ros/ros.h>
#include "siasun_dualarm_msgs/IORW.h"
#include "SetIO.h"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>
#include "industrial_moveit.h"
int main(int argc, char **argv)
{

    ros::init(argc, argv, "io_test");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    IOStatesConfigClient *pIOStatesConfigClient=IOStatesConfigClient::instance();
    bool state;
    int8_t pin=2;
    ros::Rate(100);
    while(ros::ok())
    {
        pIOStatesConfigClient->get_arml_tool_digital_in(pin,state);
        sleep(2);
        pIOStatesConfigClient->get_arml_tool_digital_out(pin,state);
        sleep(2);
        pIOStatesConfigClient->get_standard_digital_in(pin,state);
        sleep(2);
        ros::spinOnce();
    }
}


