/*
* Software License Agreement (BSD License) 
*
* Copyright (c) 2011, Southwest Research Institute
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 	* Redistributions of source code must retain the above copyright
* 	notice, this list of conditions and the following disclaimer.
* 	* Redistributions in binary form must reproduce the above copyright
* 	notice, this list of conditions and the following disclaimer in the
* 	documentation and/or other materials provided with the distribution.
* 	* Neither the name of the Southwest Research Institute, nor the names 
*	of its contributors may be used to endorse or promote products derived
*	from this software without specific prior written permission.
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

#include <algorithm>

#include "industrial_robot_client/joint_relay_handler.h"
#include "simple_message/log_wrapper.h"
#include "std_msgs/String.h"


using industrial::shared_types::shared_real;
using industrial::smpl_msg_connection::SmplMsgConnection;
using namespace industrial::simple_message;
using industrial_robot_client::siasun_dualarm_utils::getJointGroups;

namespace industrial_robot_client
{
namespace joint_relay_handler
{

bool JointRelayHandler::init(SmplMsgConnection* connection, std::vector<std::string>& joint_names)
{
    this->robot_collision_=node_.advertise<std_msgs::String>("robot_collision",1);
    this->pub_joint_control_state_ =
            this->node_.advertise<control_msgs::FollowJointTrajectoryFeedback>("feedback_states", 1);

    this->pub_joint_sensor_state_ = this->node_.advertise<sensor_msgs::JointState>("joint_states",1);

    // save "complete" joint-name list, preserving any blank entries for later use
    this->all_joint_names_ = joint_names;

    robot_ip_= ROBOT_IP;

    return init((int)StandardMsgTypes::JOINT, connection);
}
void JointRelayHandler::callBackIp(const std_msgs::Int32::ConstPtr &msg)
{
    robot_ip_= msg->data;
}

bool JointRelayHandler::init(SmplMsgConnection* connection,  std::vector<std::string>& joint_names ,std::map<int, RobotGroup> &robot_groups)
{
    this->robot_groups_ = robot_groups;
    this->all_joint_names_ = joint_names;
    for (std::map<int, RobotGroup>::iterator  iterator= robot_groups.begin(); iterator != robot_groups.end(); iterator++)
    {
        std::string name_str, ns_str;
        int robot_id = iterator->first;
        name_str = iterator->second.get_name();
        ros::Publisher pub_joint_control_state;
        ros::Publisher pub_joint_sensor_state;
        ns_str = iterator->second.get_ns();
        pub_joint_control_state=this->node_.advertise<control_msgs::FollowJointTrajectoryFeedback>(ns_str + "/" + name_str + "/feedback_states", 1);
        pub_joint_sensor_state = this->node_.advertise<sensor_msgs::JointState>(ns_str + "/" + name_str + "/joint_states", 1);
        this->pub_joint_control_state_map_[robot_id] = pub_joint_control_state;
        this->pub_joint_sensor_state_map_ [robot_id]= pub_joint_sensor_state;
        all_joint_names_map_[robot_id]=iterator->second.get_joint_names();

    }
    this->pub_joint_control_state_ = this->node_.advertise<control_msgs::FollowJointTrajectoryFeedback>("feedback_states", 1);
    this->pub_joint_sensor_state_ = this->node_.advertise<sensor_msgs::JointState>("joint_states",1);
    this->robotIp_=this->node_.subscribe("robot_ip",100, &JointRelayHandler::callBackIp,this);
    this->robot_collision_=node_.advertise<std_msgs::String>("robot_collision",1);
    robot_ip_= ROBOT_IP;
    return init((int)StandardMsgTypes::JOINT, connection);
}


bool JointRelayHandler::internalCB(SimpleMessage& in)
{
    JointMessage joint_msg;

    if (!joint_msg.init(in))
    {
        LOG_ERROR("Failed to initialize joint message");
        return false;

    }
    return internalCB(joint_msg);
}

bool JointRelayHandler::internalCB(JointMessage& in)
{
    control_msgs::FollowJointTrajectoryFeedback control_state;
    sensor_msgs::JointState sensor_state;
    static int iOnceWarming=0;
    static robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    static robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    static planning_scene::PlanningScene planning_scene(kinematic_model);
    static collision_detection::CollisionRequest collision_request;
    static collision_detection::CollisionResult collision_result;
    static collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
    static std::vector<double> vCollisionTemp(14);
    static  std::vector<double>vRobotCollision(14);
    static std_msgs::String msg;
    robot_state::RobotState& copied_state = planning_scene.getCurrentStateNonConst();
    bool rtn = true;
    if (create_messages(in, &control_state, &sensor_state))
    {
        this->pub_joint_control_state_.publish(control_state);
        this->pub_joint_sensor_state_.publish(sensor_state);
        try
        {
        double dSum=0;
        for(int i=0;i<sensor_state.position.size();i++)
            vRobotCollision[i]=sensor_state.position[i];
        copied_state.setJointGroupPositions("arms",vRobotCollision);
        collision_result.clear();
        planning_scene.checkCollision(collision_request, collision_result, copied_state, acm);
        for(unsigned int i=0;i<vRobotCollision.size();i++)
        {
            dSum=fabs(vRobotCollision[i]-vCollisionTemp[i])+dSum;
        }
        if(dSum>0.1)
            iOnceWarming=0;
        if(collision_result.collision && 0==iOnceWarming)
        {
            vCollisionTemp=vRobotCollision;
            iOnceWarming=1;
            msg.data="Collision Has Found! Will Stop Robot";
            robot_collision_.publish(msg);
            //ROS_ERROR("Collision Has Found! Will Stop Robot\n");
        }
        else if(!collision_result.collision && 1==iOnceWarming)
            iOnceWarming=0;
        }
        catch(...)
        {
            ROS_WARN("robot_collision check failed!");
        }

    }
    else
        rtn = false;
    if(!this->robot_groups_.empty())
    {
        for( std::map<int, RobotGroup>::iterator it=robot_groups_.begin();it!=robot_groups_.end();++it)
        {
            int robot_ip=it->first;
            control_msgs::FollowJointTrajectoryFeedback control_state_ip;
            sensor_msgs::JointState sensor_state_ip;
            if (create_messages(in, &control_state_ip, &sensor_state_ip,robot_ip))
            {
                this->pub_joint_control_state_map_[robot_ip].publish(control_state_ip);
                this->pub_joint_sensor_state_map_[robot_ip].publish(sensor_state_ip);
            }
            else
                rtn = false;
        }

    }
    if (CommTypes::SERVICE_REQUEST == in.getMessageType())
    {
        SimpleMessage reply;
        in.toReply(reply, rtn ? ReplyTypes::SUCCESS : ReplyTypes::FAILURE);
        this->getConnection()->sendMsg(reply);
    }
    return rtn;


}

// TODO: Add support for other message fields (velocity, effort, desired pos)
bool JointRelayHandler::create_messages(JointMessage& msg_in,
                                        control_msgs::FollowJointTrajectoryFeedback* control_state,
                                        sensor_msgs::JointState* sensor_state)
{
    // read joint positions from JointMessage
    std::vector<double> all_joint_pos(all_joint_names_.size());
    for (int i=0; i<all_joint_names_.size(); ++i)
    {
        shared_real value;
        if (msg_in.getJoints().getJoint(i, value))
            all_joint_pos[i] = value;
        else
            LOG_ERROR("Failed to parse #%d value from JointMessage", i);
    }

    // apply transform to joint positions, if required
    std::vector<double> xform_joint_pos;
    if (!transform(all_joint_pos, &xform_joint_pos))
    {
        LOG_ERROR("Failed to transform joint positions");
        return false;
    }

    // select specific joints for publishing
    std::vector<double> pub_joint_pos;
    std::vector<std::string> pub_joint_names;
    if (!select(xform_joint_pos, all_joint_names_, &pub_joint_pos, &pub_joint_names))
    {
        LOG_ERROR("Failed to select joints for publishing");
        return false;
    }

    // assign values to messages
    control_msgs::FollowJointTrajectoryFeedback tmp_control_state;  // always start with a "clean" message
    tmp_control_state.header.stamp = ros::Time::now();
    tmp_control_state.joint_names = pub_joint_names;
    tmp_control_state.actual.positions = pub_joint_pos;
    *control_state = tmp_control_state;

    sensor_msgs::JointState tmp_sensor_state;
    tmp_sensor_state.header.stamp = ros::Time::now();
    tmp_sensor_state.name = pub_joint_names;
    tmp_sensor_state.position = pub_joint_pos;
    *sensor_state = tmp_sensor_state;

    return true;
}

bool JointRelayHandler::create_messages(JointMessage& msg_in,
                                        control_msgs::FollowJointTrajectoryFeedback* control_state,
                                        sensor_msgs::JointState* sensor_state,int robot_ip)
{
    // read joint positions from JointMessage
    std::vector<double> all_joint_pos(all_joint_names_.size());
    for (int i=0; i<all_joint_names_.size(); ++i)
    {
        shared_real value;
        if (msg_in.getJoints().getJoint(i, value))
            all_joint_pos[i] = value;
        else
            LOG_ERROR("Failed to parse #%d value from JointMessage", i);
    }

    // apply transform to joint positions, if required
    std::vector<double> xform_joint_pos;
    if (!transform(all_joint_pos, &xform_joint_pos))
    {
        LOG_ERROR("Failed to transform joint positions");
        return false;
    }

    // select specific joints for publishing
    std::vector<double> pub_joint_pos;
    std::vector<std::string> pub_joint_names;
    if (!select(xform_joint_pos, all_joint_names_, &pub_joint_pos, &pub_joint_names,robot_ip))
    {
        LOG_ERROR("Failed to select joints for publishing");
        return false;
    }

    // assign values to messages
    control_msgs::FollowJointTrajectoryFeedback tmp_control_state;  // always start with a "clean" message
    tmp_control_state.header.stamp = ros::Time::now();
    tmp_control_state.joint_names = pub_joint_names;
    tmp_control_state.actual.positions = pub_joint_pos;
    *control_state = tmp_control_state;

    sensor_msgs::JointState tmp_sensor_state;
    tmp_sensor_state.header.stamp = ros::Time::now();
    tmp_sensor_state.name = pub_joint_names;
    tmp_sensor_state.position = pub_joint_pos;
    *sensor_state = tmp_sensor_state;

    return true;
}
bool JointRelayHandler::select(const std::vector<double>& all_joint_pos, const std::vector<std::string>& all_joint_names,
                               std::vector<double>* pub_joint_pos, std::vector<std::string>* pub_joint_names)
{
    ROS_ASSERT(all_joint_pos.size() == all_joint_names.size());

    pub_joint_pos->clear();
    pub_joint_names->clear();

    // skip over "blank" joint names
    for (int i=0; i<all_joint_pos.size(); ++i)
    {
        if (all_joint_names[i].empty())
            continue;

        pub_joint_pos->push_back(all_joint_pos[i]);
        pub_joint_names->push_back(all_joint_names[i]);
    }

    return true;
}

bool JointRelayHandler::select(const std::vector<double>& all_joint_pos, const std::vector<std::string>& all_joint_names,
                               std::vector<double>* pub_joint_pos, std::vector<std::string>* pub_joint_names,int robot_ip)
{
    ROS_ASSERT(all_joint_pos.size() == all_joint_names.size());

    pub_joint_pos->clear();
    pub_joint_names->clear();
    std::vector<std::string> find_pub_joint_names=this->robot_groups_[robot_ip].get_joint_names();
    // skip over "blank" joint names
    for (int i=0; i<all_joint_pos.size(); ++i)
    {
        if (std::find(find_pub_joint_names.begin(),find_pub_joint_names.end(),all_joint_names[i])==find_pub_joint_names.end())
            continue;

        pub_joint_pos->push_back(all_joint_pos[i]);
        pub_joint_names->push_back(all_joint_names[i]);
    }

    return true;
}

}//namespace joint_relay_handler
}//namespace industrial_robot_client




