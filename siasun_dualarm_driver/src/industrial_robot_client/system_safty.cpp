#include "industrial_robot_client/system_safty.h"
#include "simple_message/shared_types.h"
#include "simple_message/simple_message.h"“
using industrial::smpl_msg_connection::SmplMsgConnection;
using namespace industrial::simple_message;
using namespace industrial::byte_array;
namespace StandardSocketPorts = industrial::simple_socket::StandardSocketPorts;

namespace industrial_robot_client
{
namespace system_safty
{
System_safty::System_safty()
{
    this->connection_ = NULL;
}

bool System_safty::init(std::string default_ip, int default_port)
{
    std::string ip;
    int port;
    // override IP/port with ROS params, if available
    ros::param::param<std::string>("robot_ip_address", ip, default_ip);
    ros::param::param<int>("~port", port, default_port);

    // check for valid parameter values
    if (ip.empty())
    {
        ROS_ERROR("No valid robot IP address found.  Please set ROS 'robot_ip_address' param");
        return false;
    }
    if (port <= 0)
    {
        ROS_ERROR("No valid robot IP port found.  Please set ROS '~port' param");
        return false;
    }

    char* ip_addr = strdup(ip.c_str());  // connection.init() requires "char*", not "const char*"
    ROS_INFO("Robot state connecting to IP address: '%s:%d'", ip_addr, port);
    default_tcp_connection_.init(ip_addr, port);
    free(ip_addr);

    return init(&default_tcp_connection_);
}

bool System_safty::init(SmplMsgConnection* connection)
{

    this->connection_ = connection;
    connection_->makeConnect();
    return true;
}


void System_safty::run()
{   ros::Rate r(50);
    while(ros::ok())
    {
        SimpleMessage success;
        success.init(StandardMsgTypes::SYSTEM_SAFTY, CommTypes::SERVICE_REPLY, ReplyTypes::SUCCESS);
        connection_->sendMsg(success);
        r.sleep();

    }
}

}
}
