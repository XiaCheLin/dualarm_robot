#include "SetIO.h"

boost::mutex IOStatesConfigClient::mutex_;
IOStatesConfigClient* IOStatesConfigClient::p=NULL;
IOStatesConfigClient* IOStatesConfigClient::instance()
{
    if (p == NULL)
        {
            mutex_.lock();
            if (p == NULL)
                p = new IOStatesConfigClient();
            mutex_.unlock();
        }
        return p;

}
IOStatesConfigClient::IOStatesConfigClient()
{
    IOclient_=nh_.serviceClient<siasun_dualarm_msgs::IORW>("IO_states_config");
}
IOError IOStatesConfigClient::get_standard_analog_in(int8_t pin, float &state)
{
    mutex_.lock();
    srv_.request.fun=1;
    srv_.request.pin=pin;
    srv_.request.state=0;
    if(!IOclient_.call(srv_))
    {
        ROS_ERROR("IO OPERATION FAILED!");
    }
    else
    state=srv_.response.state;
      mutex_.unlock();
    return srv_.response.success;

}
IOError IOStatesConfigClient::get_standard_analog_out(int8_t pin, float &state)
{
    mutex_.lock();
    srv_.request.fun=2;
    srv_.request.pin=pin;
    srv_.request.state=0;
    if(!IOclient_.call(srv_))
    {
        ROS_ERROR("IO OPERATION FAILED!");
    }
    else
    state=srv_.response.state;
      mutex_.unlock();
    return srv_.response.success;

}
IOError IOStatesConfigClient::get_standard_digital_in(int8_t pin, bool &state)
{
    mutex_.lock();
    srv_.request.fun=3;
    srv_.request.pin=pin;
    srv_.request.state=0;
    if(!IOclient_.call(srv_))
    {
        ROS_ERROR("IO OPERATION FAILED!");
    }
    else
    state=srv_.response.state==1?true:false;
    mutex_.unlock();
    return srv_.response.success;

}
IOError IOStatesConfigClient::get_standard_digital_out(int8_t pin,bool  &state)
{
    mutex_.lock();
    srv_.request.fun=4;
    srv_.request.pin=pin;
    srv_.request.state=0;
    if(!IOclient_.call(srv_))
    {
        ROS_ERROR("IO OPERATION FAILED!");
    }
    else
     state=srv_.response.state==1?true:false;
    mutex_.unlock();
    return srv_.response.success;
}
IOError IOStatesConfigClient::set_standard_analog_out(int8_t pin,float state)
{
    mutex_.lock();
    srv_.request.fun=5;
    srv_.request.pin=pin;
    srv_.request.state=state;
    if(!IOclient_.call(srv_))
    {
        ROS_ERROR("IO OPERATION FAILED!");
    }
    mutex_.unlock();
    return srv_.response.success;
}
IOError IOStatesConfigClient::set_standard_digital_out(int8_t pin,bool state)
{
    mutex_.lock();
    srv_.request.fun=6;
    srv_.request.pin=pin;
    srv_.request.state=state?1:0;
    if(!IOclient_.call(srv_))
    {
        ROS_ERROR("IO OPERATION FAILED!");
    }
    mutex_.unlock();
    return srv_.response.success;
}
IOError IOStatesConfigClient::get_arml_tool_analog_in(int8_t pin, float &state)
{
    mutex_.lock();
    srv_.request.fun=7;
    srv_.request.pin=pin;
    srv_.request.state=0;
    if(!IOclient_.call(srv_))
    {
        ROS_ERROR("IO OPERATION FAILED!");
    }
    else
    state=srv_.response.state;
      mutex_.unlock();
    return srv_.response.success;

}
IOError IOStatesConfigClient::get_arml_tool_analog_out(int8_t pin, float &state)
{
    mutex_.lock();
    srv_.request.fun=8;
    srv_.request.pin=pin;
    srv_.request.state=0;
    if(!IOclient_.call(srv_))
    {
        ROS_ERROR("IO OPERATION FAILED!");
    }
    else
    state=srv_.response.state;
      mutex_.unlock();
    return srv_.response.success;

}
IOError IOStatesConfigClient::get_arml_tool_digital_in(int8_t pin, bool &state)
{
    mutex_.lock();
    srv_.request.fun=9;
    srv_.request.pin=pin;
    srv_.request.state=0;
    if(!IOclient_.call(srv_))
    {
        ROS_ERROR("IO OPERATION FAILED!");
    }
    else
    state=srv_.response.state==1?true:false;
    mutex_.unlock();
    return srv_.response.success;
}
IOError IOStatesConfigClient::get_arml_tool_digital_out(int8_t pin,bool  &state)
{

    mutex_.lock();
    srv_.request.fun=10;
    srv_.request.pin=pin;
    srv_.request.state=0;
    if(!IOclient_.call(srv_))
    {
        ROS_ERROR("IO OPERATION FAILED!");
    }
    else
    state=srv_.response.state==1?true:false;
    mutex_.unlock();
    return srv_.response.success;
}
IOError IOStatesConfigClient::set_arml_tool_analog_out(int8_t pin, float state)
{
    mutex_.lock();
    srv_.request.fun=11;
    srv_.request.pin=pin;
    srv_.request.state=state;
    if(!IOclient_.call(srv_))
    {
        ROS_ERROR("IO OPERATION FAILED!");
    }
    mutex_.unlock();
    return srv_.response.success;
}
IOError IOStatesConfigClient::set_arml_tool_digital_out(int8_t pin, bool state)
{
    mutex_.lock();
    srv_.request.fun=12;
    srv_.request.pin=pin;
    srv_.request.state=state;
    if(!IOclient_.call(srv_))
    {
        ROS_ERROR("IO OPERATION FAILED!");
    }
    mutex_.unlock();
    return srv_.response.success;
}
IOError IOStatesConfigClient::get_armr_tool_analog_in(int8_t pin, float &state)
{
    mutex_.lock();
    srv_.request.fun=13;
    srv_.request.pin=pin;
    srv_.request.state=0;
    if(!IOclient_.call(srv_))
    {
        ROS_ERROR("IO OPERATION FAILED!");
    }
    else
    state=srv_.response.state;
      mutex_.unlock();
    return srv_.response.success;
}
IOError IOStatesConfigClient::get_armr_tool_analog_out(int8_t pin, float &state)
{
    mutex_.lock();
    srv_.request.fun=14;
    srv_.request.pin=pin;
    srv_.request.state=0;
    if(!IOclient_.call(srv_))
    {
        ROS_ERROR("IO OPERATION FAILED!");
    }
    else
    state=srv_.response.state;
      mutex_.unlock();
    return srv_.response.success;
}
IOError IOStatesConfigClient::get_armr_tool_digital_in(int8_t pin,bool &state)
{
    mutex_.lock();
    srv_.request.fun=15;
    srv_.request.pin=pin;
    srv_.request.state=0;
    if(!IOclient_.call(srv_))
    {
        ROS_ERROR("IO OPERATION FAILED!");
    }
    else
    state=srv_.response.state==1?true:false;
    mutex_.unlock();
    return srv_.response.success;
}
IOError IOStatesConfigClient::get_armr_tool_digital_out(int8_t pin, bool &state)
{
    mutex_.lock();
    srv_.request.fun=16;
    srv_.request.pin=pin;
    srv_.request.state=0;
    if(!IOclient_.call(srv_))
    {
        ROS_ERROR("IO OPERATION FAILED!");
    }
    else
    state=srv_.response.state==1?true:false;
    mutex_.unlock();
    return srv_.response.success;
}
IOError IOStatesConfigClient::set_armr_tool_analog_out(int8_t pin, float state)
{
    mutex_.lock();
    srv_.request.fun=17;
    srv_.request.pin=pin;
    srv_.request.state=state;
    if(!IOclient_.call(srv_))
    {
        ROS_ERROR("IO OPERATION FAILED!");
    }
    mutex_.unlock();
    return srv_.response.success;
}
IOError IOStatesConfigClient::set_armr_tool_digital_out(int8_t pin,bool  state)
{
    mutex_.lock();
    srv_.request.fun=18;
    srv_.request.pin=pin;
    srv_.request.state=state;
    if(!IOclient_.call(srv_))
    {
        ROS_ERROR("IO OPERATION FAILED!");
    }
    mutex_.unlock();
    return srv_.response.success;
}

