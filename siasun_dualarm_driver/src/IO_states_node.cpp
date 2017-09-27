#include "industrial_robot_client/IO_states_config.h"
using namespace industrial_robot_client::IO_states_config;
int main(int argc, char** argv)
{
    ros::init(argc,argv,"IO_states_config");
    IOStatesConfig ioStatesConfig;
    ioStatesConfig.init("192.168.12.230");
    ioStatesConfig.run();
    return 0;
}
