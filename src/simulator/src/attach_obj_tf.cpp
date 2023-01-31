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

std::string ROBOT_NAME = "undef";

bool g_attach=false;
std::string g_obj="";

void command_cb(const std_msgs::String& msg)
{
    size_t sep_pos = msg.data.find_first_of(" ");
    std::string cmd = msg.data.substr(0, sep_pos);
    std::string obj = msg.data.substr(sep_pos+1, std::string::npos);

    if(cmd=="grab")
    {
        if(g_attach)
            ROS_ERROR("Trying to grap object %s but robot is already holding object %s!", obj.c_str(), g_obj.c_str());
        else
        {
            ROS_INFO("%s has grabbed %s.", ROBOT_NAME.c_str(), obj.c_str());
            g_attach = true;
            g_obj = obj;
        }
    }
    else if(cmd=="drop")
    {
        if(!g_attach)
            ROS_ERROR("Trying to drop but robot isn't holding any object!");
        else
        {
            ROS_INFO("%s has dropped %s.", ROBOT_NAME.c_str(), g_obj.c_str());
            g_attach = false;
        }
    }
    else
        ROS_ERROR("Command %s unknown!", cmd.c_str());
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

    ros::Subscriber cmd_sub = node_handle.subscribe("/"+ROBOT_NAME+"_attach_cmd", 10, command_cb);

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

            // std::cout << "(" << pose.position.x << "," << pose.position.y << "," << pose.position.z << ") ("
                // << pose.orientation.w << "," << pose.orientation.x << "," << pose.orientation.y << "," << pose.orientation.z << ")" <<  std::endl;

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