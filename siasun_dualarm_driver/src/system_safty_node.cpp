#include "industrial_robot_client/system_safty.h"
using namespace industrial_robot_client::system_safty;
int main(int argc, char** argv)
{
    ros::init(argc,argv,"system_safty");
    System_safty system_safty;
    system_safty.init("192.168.12.230");
    system_safty.run();
    return 0;
}
