/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2014, Southwest Research Institute
 * Author: Shaun Edwards
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *  notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimer in the
 *  documentation and/or other materials provided with the distribution.
 *  * Neither the name of the Southwest Research Institute, nor the names
 *  of its contributors may be used to endorse or promote products derived
 *  from this software without specific prior written permission.
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

#include "industrial_robot_client/siasun_dualarm_utils.h"
#include "ros/ros.h"

namespace industrial_robot_client
{
namespace siasun_dualarm_utils
{

bool getJointGroups(const std::string controller_param, std::map<int, RobotGroup> & robot_groups)
{
    if(ros::param::has(controller_param))
    {
        XmlRpc::XmlRpcValue controllers_list_rpc;
        ros::param::get(controller_param, controllers_list_rpc);
        if(controllers_list_rpc.size()>1)
        {
            std::vector<XmlRpc::XmlRpcValue> controllers_list;
            ROS_INFO_STREAM("Loading controller_list");
            ROS_INFO_STREAM("Found " << controllers_list_rpc.size()-1 << " additional controllers");
            for (int i = 1; i < controllers_list_rpc.size(); i++)
            {
                XmlRpc::XmlRpcValue state_value;
                state_value = controllers_list_rpc[i];
                ROS_INFO_STREAM("controller(state_value): " << state_value);
                controllers_list.push_back(state_value);
            }


            for (int i = 0; i < controllers_list.size(); i++)
            {
                ROS_INFO_STREAM("Loading group: " << controllers_list[i]);
                RobotGroup rg;
                std::vector<std::string> rg_joint_names;

                XmlRpc::XmlRpcValue joints;

                joints = controllers_list[i]["joints"];
                for (int jt = 0; jt < joints.size(); jt++)
                {
                    rg_joint_names.push_back(static_cast<std::string>(joints[jt]));
                }
                int group_number_int =i+1;

                XmlRpc::XmlRpcValue ns_name;
                std::string ns_name_string;

                ns_name = controllers_list[i]["name"];
                ns_name_string = static_cast<std::string>(ns_name);
                if(ns_name_string.find('/')==ns_name_string.npos)
                {
                    ROS_ERROR("can not find controllers_list ns and name(format:ns/name)");
                    return false;
                }
                std::string ns_string(ns_name_string.c_str(),ns_name_string.c_str()+ns_name_string.find('/'));
                std::string name_string(ns_name_string.c_str()+ns_name_string.find('/')+1,ns_name_string.c_str()+ns_name_string.length()) ;

                ROS_DEBUG_STREAM("Setting group: " );
                ROS_DEBUG_STREAM("  group number(int): " << group_number_int  );
                ROS_DEBUG_STREAM("  joints_names(size): " << rg_joint_names.size()  );
                ROS_DEBUG_STREAM("  name: " << name_string  );
                ROS_DEBUG_STREAM("  ns: " << ns_string );
                rg.set_group_id(group_number_int);
                rg.set_joint_names(rg_joint_names);
                rg.set_name(name_string);
                rg.set_ns(ns_string);

                robot_groups[group_number_int] = rg;
            }

            ROS_INFO_STREAM("Loaded " << robot_groups.size() << " groups");
            return true;
        }
        else
            return false;
    }
    else
    {
        ROS_ERROR_STREAM("Failed to find " << controller_param << " parameter");
        return false;
    }
}

} //siasun_dualarm_utils
} //industrial_robot_client

