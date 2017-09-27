/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROBOT_IO_STATES
#define ROBOT_IO_STATES

#include <vector>
#include <string>
#include "simple_message/smpl_msg_connection.h"
#include "simple_message/socket/tcp_client.h"
#include "industrial_robot_client/siasun_dualarm_utils.h"
#include "siasun_dualarm_msgs/IORW.h"
#include "boost/thread.hpp"
#include "siasun_dualarm_msgs/MoveHead.h"
namespace industrial_robot_client
{
namespace IO_states_config
{
using industrial::smpl_msg_connection::SmplMsgConnection;
using industrial::tcp_client::TcpClient;
namespace StandardSocketPorts = industrial::simple_socket::StandardSocketPorts;

enum IOReplyStates
{

    SUCCESS=0,
    UNDEFINE=1,
    FUN_NOTMATCH=2,
    IO_NOTMATCH_OR_MSGERROR=3
};

enum IOCount
{
    STANDART_DIO_out=5,
    STANDART_DIO_in=5,
    STANDART_AIO_out=0,
    STANDART_AIO_in=0,
    ARML_DIO_out=2,
    ARML_DIO_in=2,
    ARML_AIO_out=0,
    ARML_AIO_in=0,
    ARMR_DIO_out=2,
    ARMR_DIO_in=2,
    ARMR_AIO_out=0,
    ARMR_AIO_in=0,
    IOTypesNone=0,
    IOTypesIO=1,
    IOTypesHead=2,
    IOTypesCommand=3

};
/**
 * \brief Generic template that reads state-data from a robot controller
 * and publishes matching messages to various ROS topics.
 *
 * Users should replace the default class members
 * to implement robot-specific behavior.
 */
//* RobotStateInterface
class IOStatesConfig
{

public:



//#pragma pack(4)
    struct IOStates
    {
        int IOTypes;//0:none  1:IO 2:movehead 3:command
        bool IOSetDown;//set finished
        int fun;
        int pin;
        float IOState;
        //movehead
        float pan;
        float tilt;
        float vel;
        //command
        bool Stop;
    };

    IOStatesConfig();


    bool init(std::string default_ip = "", int default_port = StandardSocketPorts::IO);



    bool init(SmplMsgConnection* connection);


    void run();


    SmplMsgConnection* get_connection()
    {
        return this->connection_;
    }

    bool moveHead(siasun_dualarm_msgs::MoveHeadRequest &moveHeadrequest,siasun_dualarm_msgs::MoveHeadResponse &moveHeadresponse);

    bool IOStatesRW(siasun_dualarm_msgs::IORWRequest &IOrequest, siasun_dualarm_msgs::IORWResponse &IOresponse);

    bool get_standard_analog_in(int8_t &pin, float &state);
    bool get_standard_analog_out(int8_t &pin, float &state);
    bool get_standard_digital_in(int8_t &pin,bool &state);
    bool get_standard_digital_out(int8_t &pin,bool &state);
    bool set_standard_analog_out(int8_t &pin,float &state);
    bool set_standard_digital_out(int8_t &pin,bool &state);
    bool get_arml_tool_analog_in(int8_t &pin, float &state);
    bool get_arml_tool_analog_out(int8_t &pin, float &state);
    bool get_arml_tool_digital_in(int8_t &pin,bool &state);
    bool get_arml_tool_digital_out(int8_t &pin,bool &state);
    bool set_arml_tool_analog_out(int8_t &pin,float &state);
    bool set_arml_tool_digital_out(int8_t &pin, bool &state);
    bool get_armr_tool_analog_in(int8_t &pin, float &state);
    bool get_armr_tool_analog_out(int8_t &pin, float &state);
    bool get_armr_tool_digital_in(int8_t &pin,bool &state);
    bool get_armr_tool_digital_out(int8_t &pin,bool &state);
    bool set_armr_tool_analog_out(int8_t &pin,float &state);
    bool set_armr_tool_digital_out(int8_t &pin,bool &state);
    bool get_analog(int8_t &pin, float &state);
    bool get_digital(int8_t &pin,bool &state);
    bool set_analog(int8_t &pin, float &state);
    bool set_digital(int8_t &pin,bool &state);
protected:
    TcpClient default_tcp_connection_;
    SmplMsgConnection* connection_;
    ros::ServiceServer io_srv_;
    ros::ServiceServer move_head_;
    struct IOStates io_states_;
    ros::NodeHandle nh_;
    boost::mutex mutex_;



};


}
}//industrial_robot_cliet


#endif /* ROBOT_IO_STATES_H */

