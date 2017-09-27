#include "industrial_robot_client/IO_states_config.h"
#include "simple_message/shared_types.h"
#include "simple_message/simple_message.h"
using industrial::smpl_msg_connection::SmplMsgConnection;
using namespace industrial::simple_message;
using namespace industrial::byte_array;
#define PANPOSLIMIT 90
#define TILTPOSLIMIT 130
namespace StandardSocketPorts = industrial::simple_socket::StandardSocketPorts;

namespace industrial_robot_client
{
namespace IO_states_config
{

IOStatesConfig::IOStatesConfig()
{
    this->connection_ = NULL;
    io_srv_ = nh_.advertiseService("IO_states_config",&IOStatesConfig::IOStatesRW, this);
    move_head_=nh_.advertiseService("move_head",&IOStatesConfig::moveHead,this);
}


bool IOStatesConfig::init(std::string default_ip, int default_port)
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

bool IOStatesConfig::init(SmplMsgConnection* connection)
{

    this->connection_ = connection;
    connection_->makeConnect();
    return true;
}

bool IOStatesConfig::moveHead(siasun_dualarm_msgs::MoveHeadRequest &moveHeadrequest,siasun_dualarm_msgs::MoveHeadResponse &moveHeadresponse)
{
    if(moveHeadrequest.pan<0 || moveHeadrequest.pan>PANPOSLIMIT || moveHeadrequest.tilt<0 || moveHeadrequest.tilt>TILTPOSLIMIT)
    {
        ROS_ERROR("moveHead pos excess limit pos!");
        moveHeadresponse.success=false;
        return false;
    }
    io_states_.IOTypes=IOCount::IOTypesHead;
    io_states_.pan=moveHeadrequest.pan;
    io_states_.tilt=moveHeadrequest.tilt;
    io_states_.vel=moveHeadrequest.vel;
    io_states_.bMoveHeadExecute=true;
    if (!this->connection_->isConnected())
    {
        ROS_WARN("Attempting robot reconnection");
        this->connection_->makeConnect();
    }
    SimpleMessage msg,replay;
    ByteArray bytearray;
    char buff[100];
    memcpy(buff,&io_states_,sizeof(io_states_));
    if(!bytearray.init(buff,sizeof(io_states_)))
    {
        moveHeadresponse.success=-1;
        return false;
    }
    if(!msg.init(22,1,0,bytearray))
    {
        moveHeadresponse.success=-1;
        return false;
    }
    if(!connection_->sendAndReceiveMsg(msg,replay))
    {
        moveHeadresponse.success=-1;
        return false;
    }
    moveHeadresponse.success=0;
    io_states_.bMoveHeadExecute=false;
    return true;
}


bool IOStatesConfig::IOStatesRW(siasun_dualarm_msgs::IORWRequest &IOrequest,siasun_dualarm_msgs::IORWResponse &IOresponse)
{
    mutex_.lock();
    if(IOrequest.fun<12 && IOrequest.fun>0 &&IOrequest.state>=0)
    {
        io_states_.fun=IOrequest.fun;
        io_states_.pin=IOrequest.pin;
        switch (IOrequest.fun)
        {
        //IOrequest.GET_STANDARD_ANALOG_IN
        case 1:
        {
            if(STANDART_AIO_in<=0)
            {
                ROS_WARN("STANDARD_ANALOG_IN NOT DEFINE");
                IOresponse.state=0;
                io_states_.IOState=0;
                IOresponse.success=UNDEFINE;
                break;
            }
            else
            {
                float analogState;
                if(!this->get_standard_analog_in(IOrequest.pin,analogState))
                {
                    IOresponse.state=0;
                    io_states_.IOState=0;
                    IOresponse.success=IO_NOTMATCH_OR_MSGERROR;
                    break;
                }
                io_states_.IOState=analogState;
                IOresponse.state=io_states_.IOState;
                IOresponse.success=SUCCESS;
                break;
            }
        }
            //IOrequest.GET_STANDARD_ANALOG_OUT
        case 2:
        {
            if(STANDART_AIO_out<=0)
            {
                ROS_WARN("STANDARD_ANALOG_OUT NOT DEFINE");
                IOresponse.state=0;
                io_states_.IOState=0;
                IOresponse.success=UNDEFINE;
                break;
            }
            else
            {
                float analogState;
                if(!this->get_standard_analog_out(IOrequest.pin,analogState))
                {
                    IOresponse.state=0;
                    io_states_.IOState=0;
                    IOresponse.success=IO_NOTMATCH_OR_MSGERROR;
                    break;
                }
                io_states_.IOState=analogState;
                IOresponse.state=io_states_.IOState;
                IOresponse.success=SUCCESS;
                break;

            }
        }
            //IOrequest.GET_STANDARD_DIGITAL_IN
        case 3:
        {
            if(STANDART_DIO_in<=0)
            {
                ROS_WARN("STANDARD_DIGITAL_IN NOT DEFINE");
                IOresponse.state=0;
                io_states_.IOState=0;
                IOresponse.success=UNDEFINE;
                break;
            }
            else
            {
                bool digitalState;
                if(!this->get_standard_digital_in( IOrequest.pin,digitalState))
                {
                    IOresponse.state=0;
                    io_states_.IOState=0;
                    IOresponse.success=IO_NOTMATCH_OR_MSGERROR;
                    break;
                }
                io_states_.IOState=digitalState? 1:0;
                IOresponse.state=io_states_.IOState;
                IOresponse.success=SUCCESS;
                break;
            }
        }
            //IOrequest.GET_STANDARD_DIGITAL_OUT
        case 4:
        {
            if(STANDART_DIO_out<=0)
            {
                ROS_WARN("STANDARD_DIGITAL_OUT NOT DEFINE");
                IOresponse.state=0;
                io_states_.IOState=0;
                IOresponse.success=UNDEFINE;
                break;
            }
            else
            {
                bool digitalState;
                if(!this->get_standard_digital_out( IOrequest.pin,digitalState))
                {
                    IOresponse.state=0;
                    io_states_.IOState=0;
                    IOresponse.success=IO_NOTMATCH_OR_MSGERROR;
                    break;
                }
                io_states_.IOState=digitalState? 1:0;
                IOresponse.state=io_states_.IOState;
                IOresponse.success=SUCCESS;
                break;
            }
        }
            //IOrequest.SET_STANDARD_ANALOG_OUT
        case 5:
        {
            if(STANDART_AIO_out<=0)
            {
                ROS_WARN("STANDARD_ANALOG_OUT NOT DEFINE");
                IOresponse.state=0;
                IOresponse.success=UNDEFINE;
                break;
            }
            else
            {
                float analogState=IOrequest.state;
                if(!this->set_standard_analog_out(IOrequest.pin,analogState))
                {
                    IOresponse.state=0;
                    IOresponse.success=IO_NOTMATCH_OR_MSGERROR;
                }
                break;
            }
        }
            //IOrequest.SET_STANDARD_DIGITAL_OUT
        case 6:
        {
            if(STANDART_DIO_out<=0)
            {
                ROS_WARN("STANDARD_DIGITAL_OUT NOT DEFINE");
                IOresponse.state=0;
                IOresponse.success=UNDEFINE;
                break;
            }
            else
            {
                bool state=IOrequest.state>0?true:false;
                if(!this->set_standard_digital_out(IOrequest.pin,state))
                {
                    IOresponse.state=0;
                    IOresponse.success=IO_NOTMATCH_OR_MSGERROR;
                }
                break;
            }
        }
            //IOrequest.GET_ARML_TOOL_ANALOG_IN
        case 7:
        {
            if(ARML_AIO_in<=0)
            {
                ROS_WARN("ARML_TOOL_ANALOG_IN NOT DEFINE");
                IOresponse.state=0;
                IOresponse.success=UNDEFINE;
                break;
            }
            else
            {
                float analogState;
                if(!this->get_arml_tool_analog_in(IOrequest.pin,analogState))
                {
                    IOresponse.state=0;
                    io_states_.IOState=0;
                    IOresponse.success=IO_NOTMATCH_OR_MSGERROR;
                    break;
                }
                io_states_.IOState=analogState;
                IOresponse.state=io_states_.IOState;
                IOresponse.success=SUCCESS;
                break;
            }
        }
            //GET_ARML_TOOL_ANALOG_OUT
        case 8:
        {
            if(ARML_AIO_out<=0)
            {
                ROS_WARN("ARML_TOOL_ANALOG_OUT NOT DEFINE");
                IOresponse.state=0;
                IOresponse.success=UNDEFINE;
                break;
            }
            else
            {
                float analogState;
                if(!this->get_arml_tool_analog_out(IOrequest.pin,analogState))
                {
                    IOresponse.state=0;
                    io_states_.IOState=0;
                    IOresponse.success=IO_NOTMATCH_OR_MSGERROR;
                    break;
                }
                io_states_.IOState=analogState;
                IOresponse.state=io_states_.IOState;
                IOresponse.success=SUCCESS;
                break;
            }
        }
            //IOrequest.GET_ARML_TOOL_DIGITAL_IN
        case 9:
        {
            if(ARML_DIO_in<=0)
            {
                ROS_WARN("ARML_TOOL_DIGITAL_IN NOT DEFINE");
                IOresponse.state=0;
                io_states_.IOState=0;
                IOresponse.success=IO_NOTMATCH_OR_MSGERROR;
                break;
            }
            else
            {
                bool digitalState;
                if(!this->get_arml_tool_digital_in(IOrequest.pin,digitalState))
                {
                    IOresponse.state=0;
                    io_states_.IOState=0;
                    IOresponse.success=IO_NOTMATCH_OR_MSGERROR;
                    break;
                }
                io_states_.IOState=digitalState? 1:0;
                IOresponse.state=io_states_.IOState;
                IOresponse.success=SUCCESS;
                break;
            }
        }
            //IOrequest.GET_ARML_TOOL_DIGITAL_OUT
        case 10:
        {
            if(ARML_DIO_out<=0)
            {
                ROS_WARN("ARML_TOOL_DIGITAL_OUT NOT DEFINE");
                IOresponse.state=0;
                io_states_.IOState=0;
                IOresponse.success=IO_NOTMATCH_OR_MSGERROR;
                break;
            }
            else
            {
                bool digitalState;
                if(!this->get_arml_tool_digital_in(IOrequest.pin,digitalState))
                {
                    IOresponse.state=0;
                    io_states_.IOState=0;
                    IOresponse.success=IO_NOTMATCH_OR_MSGERROR;
                    break;
                }
                io_states_.IOState=digitalState? 1:0;
                IOresponse.state=io_states_.IOState;
                IOresponse.success=SUCCESS;
                break;
            }
        }
            //IOrequest.SET_ARML_TOOL_ANALOG_OUT
        case 11:
        {
            if(ARML_AIO_out<=0)
            {
                ROS_WARN("ARML_TOOL_ANALOG_OUT NOT DEFINE");
                IOresponse.state=0;
                IOresponse.success=UNDEFINE;
                break;
            }
            else
            {
                float analogState=IOrequest.state;
                if(!this->set_arml_tool_analog_out(IOrequest.pin,analogState))
                {
                    IOresponse.state=0;
                    IOresponse.success=IO_NOTMATCH_OR_MSGERROR;
                }
                break;
            }
        }
            //IOrequest.SET_ARML_TOOL_DIGITAL_OUT
        case 12:
        {
            if(ARML_DIO_out<=0)
            {
                ROS_WARN("ARML_TOOL_DIGITAL_OUT NOT DEFINE");
                IOresponse.state=0;
                IOresponse.success=UNDEFINE;
                break;
            }
            else
            {
                bool state=IOrequest.state>0?true:false;
                if(!this->set_arml_tool_digital_out(IOrequest.pin,state))
                {
                    IOresponse.state=0;
                    IOresponse.success=IO_NOTMATCH_OR_MSGERROR;
                }
                break;
            }
        }
            //IOrequest.GET_ARMR_TOOL_ANALOG_IN
        case 13:
        {
            if(ARMR_AIO_in<=0)
            {
                ROS_WARN("ARMR_TOOL_ANALOG_IN NOT DEFINE");
                IOresponse.state=0;
                IOresponse.success=UNDEFINE;
                break;
            }
            else
            {
                float analogState;
                if(!this->get_armr_tool_analog_in(IOrequest.pin,analogState))
                {
                    IOresponse.state=0;
                    io_states_.IOState=0;
                    IOresponse.success=IO_NOTMATCH_OR_MSGERROR;
                    break;
                }
                io_states_.IOState=analogState;
                IOresponse.state=io_states_.IOState;
                IOresponse.success=SUCCESS;
                break;
            }
        }
            //GET_ARMR_TOOL_ANALOG_OUT
        case 14:
        {
            if(ARMR_AIO_out<=0)
            {
                ROS_WARN("ARMR_TOOL_ANALOG_OUT NOT DEFINE");
                IOresponse.state=0;
                IOresponse.success=UNDEFINE;
                break;
            }
            else
            {
                float analogState;
                if(!this->get_armr_tool_analog_out(IOrequest.pin,analogState))
                {
                    IOresponse.state=0;
                    io_states_.IOState=0;
                    IOresponse.success=IO_NOTMATCH_OR_MSGERROR;
                    break;
                }
                io_states_.IOState=analogState;
                IOresponse.state=io_states_.IOState;
                IOresponse.success=SUCCESS;
                break;
            }
        }
            //IOrequest.GET_ARMR_TOOL_DIGITAL_IN
        case 15:
        {
            if(ARMR_DIO_in<=0)
            {
                ROS_WARN("ARMR_TOOL_DIGITAL_IN NOT DEFINE");
                IOresponse.state=0;
                IOresponse.success=UNDEFINE;
                break;
            }
            bool digitalState;
            if(!this->get_armr_tool_digital_in(IOrequest.pin,digitalState))
            {
                IOresponse.state=0;
                io_states_.IOState=0;
                IOresponse.success=IO_NOTMATCH_OR_MSGERROR;
                break;
            }
            io_states_.IOState=digitalState? 1:0;
            IOresponse.state=io_states_.IOState;
            IOresponse.success=SUCCESS;
            break;
        }
            //IOrequest.GET_ARMR_TOOL_DIGITAL_OUT
        case 16:
        {
            if(ARMR_DIO_out<=0)
            {
                ROS_WARN("ARMR_TOOL_DIGITAL_OUT NOT DEFINE");
                IOresponse.state=0;
                IOresponse.success=UNDEFINE;
                break;
            }
            bool digitalState;
            if(!this->get_armr_tool_digital_in(IOrequest.pin,digitalState))
            {
                IOresponse.state=0;
                io_states_.IOState=0;
                IOresponse.success=IO_NOTMATCH_OR_MSGERROR;
                break;
            }
            io_states_.IOState=digitalState? 1:0;
            IOresponse.state=io_states_.IOState;
            IOresponse.success=SUCCESS;
            break;
        }
            //IOrequest.SET_ARMR_TOOL_ANALOG_OUT
        case 17:
        {
            if(ARMR_DIO_out<=0)
            {
                ROS_WARN("ARMR_TOOL_ANALOG_OUT NOT DEFINE");
                IOresponse.state=0;
                IOresponse.success=UNDEFINE;
                break;
            }
            else
            {
                float analogState=IOrequest.state;
                if(!this->set_armr_tool_analog_out(IOrequest.pin,analogState))
                {
                    IOresponse.state=0;
                    IOresponse.success=IO_NOTMATCH_OR_MSGERROR;
                }
                break;
            }
        }
            //IOrequest.SET_ARMR_TOOL_DIGITAL_OUT
        case 18:
        {
            if(ARMR_DIO_out<=0)
            {
                ROS_WARN("ARMR_TOOL_DIGITAL_OUT NOT DEFINE");
                IOresponse.state=0;
                IOresponse.success=UNDEFINE;
                break;
            }
            else
            {
                bool state=IOrequest.state>0?true:false;
                if(!this->set_armr_tool_digital_out(IOrequest.pin,state))
                {
                    IOresponse.state=0;
                    IOresponse.success=IO_NOTMATCH_OR_MSGERROR;
                }
                break;
            }
        }

        }

    }
    else
    {
        IOresponse.state=0;
        IOresponse.success=FUN_NOTMATCH;

    }
    if(0!=IOresponse.success)
    {
        ROS_ERROR("operate robot IO wrong ! wrong number is: %d",IOresponse.success);
    }
    mutex_.unlock();
    //    if(IOresponse.success!=SUCCESS)
    //        return false;
    return true;

}


bool IOStatesConfig::get_standard_analog_in(int8_t &pin, float  &state)
{
    if(pin>STANDART_AIO_in ||pin<0)
    {
        ROS_ERROR("IO Pin Does Not Match! standard_analog_in(0:%d)",STANDART_AIO_in);
        return false;
    }
    if(!this->get_analog(pin,state))
        return false;
    return true;

}
bool IOStatesConfig::get_standard_analog_out(int8_t &pin, float  &state)
{
    if(pin>STANDART_AIO_out ||pin<0)
    {
        ROS_ERROR("IO Pin Does Not Match! standard_analog_out(0:%d)",STANDART_AIO_out);
        return false;
    }
    if(!this->get_analog(pin,state))
        return false;
    return true;

}
bool IOStatesConfig::get_standard_digital_in(int8_t &pin,bool &state)
{
    if(pin>STANDART_DIO_in ||pin<0)
    {
        ROS_ERROR("IO Pin Does Not Match! standard_digital_in(0:%d)",STANDART_DIO_in);
        return false;
    }
    if(!this->get_digital(pin,state))
        return false;
    return true;

}
bool IOStatesConfig::get_standard_digital_out(int8_t &pin,bool &state)
{
    if(pin>STANDART_DIO_out ||pin<0)
    {
        ROS_ERROR("IO Pin Does Not Match! standard_digital_out(0:%d)",STANDART_DIO_in);
        return false;
    }
    if(!this->get_digital(pin,state))
        return false;
    return true;


}
bool IOStatesConfig::set_standard_analog_out(int8_t &pin,float  &state)
{
    if(pin>STANDART_AIO_out ||pin<0)
    {
        ROS_ERROR("IO Pin Does Not Match! standard_analog_out(0:%d)",STANDART_AIO_out);
        return false;
    }
    if(!this->set_analog(pin,state))
        return false;
    return true;
}

bool IOStatesConfig::set_standard_digital_out(int8_t &pin,bool &state)
{
    if(pin>STANDART_DIO_out ||pin<0)
    {
        ROS_ERROR("IO Pin Does Not Match! standard_digital_out(0:%d)",STANDART_DIO_out);
        return false;
    }
    if(!this->set_digital(pin,state))
        return false;
    return true;

}
bool IOStatesConfig::get_arml_tool_analog_in(int8_t &pin, float  &state)
{
    if(pin>ARML_AIO_in ||pin<0)
    {
        ROS_ERROR("IO Pin Does Not Match! arml_tool_analog_in(0:%d)",ARML_AIO_in);
        return false;
    }
    if(!this->get_analog(pin,state))
        return false;
    return true;

}
bool IOStatesConfig::get_arml_tool_analog_out(int8_t &pin, float  &state)
{
    if(pin>ARML_AIO_out ||pin<0)
    {
        ROS_ERROR("IO Pin Does Not Match! arml_tool_analog_out(0:%d)",ARML_AIO_out);
        return false;
    }
    if(!this->get_analog(pin,state))
        return false;
    return true;

}
bool IOStatesConfig::get_arml_tool_digital_in(int8_t &pin,bool &state)
{
    if(pin>ARML_DIO_in ||pin<0)
    {
        ROS_ERROR("IO Pin Does Not Match! arml_tool_digital_in(0:%d)",ARML_DIO_in);
        return false;
    }
    if(!this->get_digital(pin,state))
        return false;
    return true;

}
bool IOStatesConfig::get_arml_tool_digital_out(int8_t &pin,bool &state)
{
    if(pin>ARML_DIO_out ||pin<0)
    {
        ROS_ERROR("IO Pin Does Not Match! arml_tool_digital_out(0:%d)",ARML_DIO_out);
        return false;
    }
    if(!this->get_digital(pin,state))
        return false;
    return true;

}
bool IOStatesConfig::set_arml_tool_analog_out(int8_t &pin,float  &state)
{
    if(pin>ARML_AIO_out ||pin<0)
    {
        ROS_ERROR("IO Pin Does Not Match! arml_tool_analog_out(0:%d)",ARML_AIO_out);
        return false;
    }
    if(!this->set_analog(pin,state))
        return false;
    return true;

}
bool IOStatesConfig::set_arml_tool_digital_out(int8_t &pin,bool &state)
{
    if(pin>ARML_DIO_out ||pin<0)
    {
        ROS_ERROR("IO Pin Does Not Match! arml_tool_digital_out(0:%d)",ARML_DIO_out);
        return false;
    }
    if(!this->set_digital(pin,state))
        return false;
    return true;

}
bool IOStatesConfig::get_armr_tool_analog_in(int8_t &pin, float  &state)
{
    if(pin>ARMR_AIO_in ||pin<0)
    {
        ROS_ERROR("IO Pin Does Not Match! armr_tool_analog_in(0:%d)",ARMR_AIO_in);
        return false;
    }
    if(!this->get_analog(pin,state))
        return false;
    return true;

}
bool IOStatesConfig::get_armr_tool_analog_out(int8_t &pin, float  &state)
{
    if(pin>ARMR_AIO_out ||pin<0)
    {
        ROS_ERROR("IO Pin Does Not Match! armr_tool_analog_out(0:%d)",ARMR_AIO_out);
        return false;
    }
    if(!this->get_analog(pin,state))
        return false;
    return true;

}
bool IOStatesConfig::get_armr_tool_digital_in(int8_t &pin,bool &state)
{
    if(pin>ARMR_DIO_in ||pin<0)
    {
        ROS_ERROR("IO Pin Does Not Match! armr_tool_digital_in(0:%d)",ARMR_DIO_in);
        return false;
    }
    if(!this->get_digital(pin,state))
        return false;
    return true;
}
bool IOStatesConfig::get_armr_tool_digital_out(int8_t &pin,bool &state)
{
    if(pin>ARMR_DIO_out ||pin<0)
    {
        ROS_ERROR("IO Pin Does Not Match! armr_tool_digital_out(0:%d)",ARMR_DIO_out);
        return false;
    }
    if(!this->get_digital(pin,state))
        return false;
    return true;
}
bool IOStatesConfig::set_armr_tool_analog_out(int8_t &pin,float  &state)
{
    if(pin>ARMR_AIO_out ||pin<0)
    {
        ROS_ERROR("IO Pin Does Not Match! armr_tool_analog_out(0:%d)",ARMR_AIO_out);
        return false;
    }
    if(!this->set_analog(pin,state))
        return false;
    return true;


}
bool IOStatesConfig::set_armr_tool_digital_out(int8_t &pin,bool &state)
{
    if(pin>ARMR_DIO_out ||pin<0)
    {
        ROS_ERROR("IO Pin Does Not Match! armr_tool_digital_out(0:%d)",ARMR_DIO_out);
        return false;
    }
    if(!this->set_digital(pin,state))
        return false;
    return true;

}

void IOStatesConfig::run()
{    while(ros::ok())
    {

        ros::spinOnce();
    }
}

bool IOStatesConfig::get_analog(int8_t &pin, float &state)
{
    if (!this->connection_->isConnected())
    {
        ROS_WARN("Attempting robot reconnection");
        this->connection_->makeConnect();
    }
    io_states_.IOTypes=IOCount::IOTypesIO;
    io_states_.pin=pin;
    io_states_.IOState=0;
    SimpleMessage msg,replay;
    ByteArray bytearray;
    char buff[100];
    memcpy(buff,&io_states_,sizeof(io_states_));
    if(!bytearray.init(buff,sizeof(io_states_)))
        return false;
    if(!msg.init(22,1,0,bytearray))
        return false;
    if(!connection_->sendAndReceiveMsg(msg,replay))
        return false;
    if(!replay.getData().unloadFront(&io_states_,sizeof(io_states_)))
        return false;
    if(io_states_.pin!=pin)
    {
        ROS_ERROR("PIN NOT MATCH!");
        return false;
    }
    state=io_states_.IOState;
    return true;
}

bool IOStatesConfig::get_digital(int8_t &pin,bool &state)
{
    if (!this->connection_->isConnected())
    {
        ROS_WARN("Attempting robot reconnection");
        this->connection_->makeConnect();
    }
    io_states_.IOTypes=IOCount::IOTypesIO;
    io_states_.pin=pin;
    io_states_.IOState=0;
    SimpleMessage msg,replay;
    ByteArray bytearray;
    char buff[100];
    memcpy(buff,&io_states_,sizeof(io_states_));
    if(!bytearray.init(buff,sizeof(io_states_)))
        return false;
    if(!msg.init(22,1,0,bytearray))
        return false;
    if(!connection_->sendAndReceiveMsg(msg,replay))
        return false;
    if(!replay.getData().unloadFront(&io_states_,sizeof(io_states_)))
        return false;
    if(io_states_.pin!=pin)
    {
        ROS_ERROR("PIN NOT MATCH!");
        return false;
    }
    state=io_states_.IOState>0? true:false;
    return true;
}
bool IOStatesConfig::set_analog(int8_t &pin, float &state)
{
    if (!this->connection_->isConnected())
    {
        ROS_WARN("Attempting robot reconnection");
        this->connection_->makeConnect();
    }
    io_states_.IOTypes=IOCount::IOTypesIO;
    io_states_.pin=pin;
    SimpleMessage msg,replay;
    ByteArray bytearray;
    char buff[100];
    io_states_.IOState=state;
    memcpy(buff,&io_states_,sizeof(io_states_));
    if(!bytearray.init(buff,sizeof(io_states_)))
        return false;
    if(!msg.init(22,1,0,bytearray))
        return false;
    if(!connection_->sendAndReceiveMsg(msg,replay))
        return false;
    return true;

}
bool IOStatesConfig::set_digital(int8_t &pin,bool &state)
{
    if (!this->connection_->isConnected())
    {
        ROS_WARN("Attempting robot reconnection");
        this->connection_->makeConnect();
    }
    io_states_.IOTypes=IOCount::IOTypesIO;
    io_states_.pin=pin;
    SimpleMessage msg,replay;
    ByteArray bytearray;
    char buff[100];
    io_states_.IOState= state? 1:0;
    memcpy(buff,&io_states_,sizeof(io_states_));
    if(!bytearray.init(buff,sizeof(io_states_)))
        return false;
    if(!msg.init(22,1,0,bytearray))
        return false;
    if(!connection_->sendAndReceiveMsg(msg,replay))
        return false;
    return true;

}





}
}
