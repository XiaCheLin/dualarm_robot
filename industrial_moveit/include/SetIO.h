
#include <ros/ros.h>
#include <urdf/model.h>
#include<unistd.h>
#include<sys/ipc.h>
#include "siasun_dualarm_msgs/IORW.h"
#include "boost/thread.hpp"

typedef int IOError;

class IOStatesConfigClient
{
public:
     static IOStatesConfigClient* instance();
    IOError get_standard_analog_in(int8_t pin, float &state);
    IOError get_standard_analog_out(int8_t pin, float &state);
    IOError get_standard_digital_in(int8_t pin,bool  &state);
    IOError get_standard_digital_out(int8_t pin, bool &state);
    IOError set_standard_analog_out(int8_t pin,float state);
    IOError set_standard_digital_out(int8_t pin, bool state);
    IOError get_arml_tool_analog_in(int8_t pin, float &state);
    IOError get_arml_tool_analog_out(int8_t pin, float &state);
    IOError get_arml_tool_digital_in(int8_t pin,bool &state);
    IOError get_arml_tool_digital_out(int8_t pin, bool &state);
    IOError set_arml_tool_analog_out(int8_t pin,float state);
    IOError set_arml_tool_digital_out(int8_t pin, bool state);
    IOError get_armr_tool_analog_in(int8_t pin, float &state);
    IOError get_armr_tool_analog_out(int8_t pin, float &state);
    IOError get_armr_tool_digital_in(int8_t pin, bool &state);
    IOError get_armr_tool_digital_out(int8_t pin,bool  &state);
    IOError set_armr_tool_analog_out(int8_t pin,float state);
    IOError set_armr_tool_digital_out(int8_t pin, bool state);


private:
    IOStatesConfigClient();
    IOStatesConfigClient(const IOStatesConfigClient &){}
    IOStatesConfigClient& operator=(const IOStatesConfigClient & client){}
    ros::ServiceClient IOclient_;
    siasun_dualarm_msgs::IORW srv_;
    static boost::mutex mutex_;
    ros::NodeHandle nh_;
    static IOStatesConfigClient *p;
};
