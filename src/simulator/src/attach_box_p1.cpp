#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "attach_p1");
    ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group_interface("panda1_arm");

    ros::Publisher gazebo_model_state_pub = node_handle.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);

    ros::Rate loop(100);

    while(ros::ok())
    {
        geometry_msgs::PoseStamped pose_EE = move_group_interface.getCurrentPose();

        geometry_msgs::Pose pose;
        pose.position.x = pose_EE.pose.position.x;
        pose.position.y = pose_EE.pose.position.y;
        pose.position.z = pose_EE.pose.position.z;
        pose.orientation.w = pose_EE.pose.orientation.w;
        pose.orientation.x = pose_EE.pose.orientation.x;
        pose.orientation.y = pose_EE.pose.orientation.y;
        pose.orientation.z = pose_EE.pose.orientation.z;

        gazebo_msgs::ModelState model_state;
        model_state.model_name = std::string("dyn_obj_1");
        model_state.pose = pose;
        model_state.reference_frame = std::string("world");

        gazebo_model_state_pub.publish(model_state);

        loop.sleep();
    }

    return 0;
}