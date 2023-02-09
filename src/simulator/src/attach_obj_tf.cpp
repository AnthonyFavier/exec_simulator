#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include "sim_msgs/AttachObj.h"
#include <std_srvs/Empty.h>

std::string ROBOT_NAME = "undef";

bool g_attach=false;
std::string g_obj="";

bool cmd_server(sim_msgs::AttachObj::Request &req, sim_msgs::AttachObj::Response &res)
{
    res.success = true;
    if(req.type==req.GRAB)
    {
        if(g_attach)
        {
            ROS_ERROR("Trying to grap object %s but robot is already holding object %s!", req.obj_name.c_str(), g_obj.c_str());
            res.success = false;
        }
        else
        {
            ROS_INFO("%s has grabbed %s.", ROBOT_NAME.c_str(), req.obj_name.c_str());
            g_attach = true;
            g_obj = req.obj_name;
        }
    }
    else if(req.type==req.DROP)
    {
        if(!g_attach)
        {
            ROS_ERROR("Trying to drop but robot isn't holding any object!");
            res.success = false;
        }
        else
        {
            ROS_INFO("%s has dropped %s.", ROBOT_NAME.c_str(), g_obj.c_str());
            g_attach = false;
            res.obj_dropped_name = g_obj;
        }
    }
    else
    {
        ROS_ERROR("Command %d unknown!", req.type);
        res.success = false;
    }

    ros::Duration(0.1).sleep();
    return true;
}

bool reset_server(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    g_attach = false;
    return true;
}

int main(int argc, char **argv)
{
    if(argc<2)
        throw std::invalid_argument("Attach_obj: Missing robot name argument!");
    else
        ROBOT_NAME = argv[1];

    ros::init(argc, argv, "attach_obj_"+ROBOT_NAME);
    ros::NodeHandle node_handle;
    
    ros::AsyncSpinner spinner(1);
    spinner.start();


    tf::TransformListener listener;

    moveit::planning_interface::MoveGroupInterface move_group_interface(ROBOT_NAME+"_arm");

    ros::Publisher gazebo_model_state_pub = node_handle.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);

    ros::ServiceServer cmd_service = node_handle.advertiseService("attach_obj", cmd_server);
    ros::ServiceServer reset_service = node_handle.advertiseService("attach_reset", reset_server);

    ros::Rate loop(200);

    while(ros::ok())
    {
        if(g_attach)
        {
            tf::StampedTransform transform;
            listener.lookupTransform("/world", "/"+ROBOT_NAME+"_grasptarget", ros::Time(0), transform);

            geometry_msgs::Pose pose;
            pose.position.x = transform.getOrigin().x();
            pose.position.y = transform.getOrigin().y();
            pose.position.z = transform.getOrigin().z();
            pose.orientation.w = transform.getRotation().w();
            pose.orientation.x = transform.getRotation().x();
            pose.orientation.y = transform.getRotation().y();
            pose.orientation.z = transform.getRotation().z();

            gazebo_msgs::ModelState model_state;
            model_state.model_name = std::string(g_obj);
            model_state.pose = pose;
            model_state.reference_frame = std::string("world");

            gazebo_model_state_pub.publish(model_state);
        }

        loop.sleep();
    }

    return 0;
}