#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

static const std::string PLANNING_GROUP_ARM = "panda_arm";
static const double PANDA_ARM_TO_HAND_OFFSET = 0.12;
static const double PANDA_HAND_TO_FINGER_OFFSET = 0.04;
ros::Publisher gazebo_model_state_pub;
robot_model::RobotModelPtr kinematic_model;
robot_state::RobotStatePtr kinematic_state;
int count = 0;
bool first=true;
geometry_msgs::Pose offset;

// moveit::planning_interface::MoveGroupInterface* move_group_interface;




void jointStatesCallback(const gazebo_msgs::LinkStates &link_states_current)
{
    //   const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("panda_arm");
    //   const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

    //   std::vector<double> joint_states;
    //   for (size_t i = 0; i < joint_states_current.position.size() - 2; ++i)
    //   {
    //     joint_states.push_back(joint_states_current.position[i + 2]);
    //   }
    //   kinematic_state->setJointGroupPositions(joint_model_group, joint_states);

    //   const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("panda_link8");

    //   double end_effector_z_offset = PANDA_ARM_TO_HAND_OFFSET + PANDA_HAND_TO_FINGER_OFFSET;
    //   Eigen::Affine3d tmp_transform(Eigen::Translation3d(Eigen::Vector3d(0.0, 0.0, end_effector_z_offset)));

    //   Eigen::Affine3d newState = end_effector_state * tmp_transform;

    // Get obj_1 state

    // if (count < 10)
    //     count++;
    // else
    // {
    //     count = 0;

        // int obj1_id = -1;
        // int obj2_id = -1;
        // int EE_id = -1;
        // for (int i = 0; i < link_states_current.name.size(); i++)
        // {
        //     if (link_states_current.name[i] == "dyn_obj_1::link_0")
        //         obj1_id = i;
        //     else if (link_states_current.name[i] == "dyn_obj_2::link_0")
        //         obj2_id = i;
        //     else if (link_states_current.name[i] == "panda::panda_link7")
        //         EE_id = i;
        //     if (obj1_id != -1 && obj2_id != -1 && EE_id!=-1)
        //         break;
        // }

        // if(first)
        // {
        //     first = false;

        //     std::cout << "ID=" << EE_id << " (" << link_states_current.pose[EE_id].position.x << "," << link_states_current.pose[EE_id].position.y << "," << link_states_current.pose[EE_id].position.z << std::endl;
        //     // offset.position.x = link_states_current.pose[obj2_id].position.x - link_states_current.pose[obj1_id].position.x;
        //     // offset.position.y = link_states_current.pose[obj2_id].position.y - link_states_current.pose[obj1_id].position.y;
        //     // offset.position.z = link_states_current.pose[obj2_id].position.z - link_states_current.pose[obj1_id].position.z;
        // }

        // geometry_msgs::PoseStamped pose_EE = move_group_interface->getCurrentPose();
        geometry_msgs::PoseStamped pose_EE ;
        geometry_msgs::Pose pose;
        pose.position.x = pose_EE.pose.position.x + offset.position.x;
        pose.position.y = pose_EE.pose.position.y + offset.position.y;
        pose.position.z = pose_EE.pose.position.z + offset.position.z;
        pose.orientation.w = pose_EE.pose.orientation.w;
        pose.orientation.x = pose_EE.pose.orientation.x;
        pose.orientation.y = pose_EE.pose.orientation.y;
        pose.orientation.z = pose_EE.pose.orientation.z;

        gazebo_msgs::ModelState model_state;
        model_state.model_name = std::string("dyn_obj_1");
        model_state.pose = pose;
        model_state.reference_frame = std::string("world");

        gazebo_model_state_pub.publish(model_state);
    // }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "box_publisher_node");
    ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group_interface("panda_arm");

    // geometry_msgs::PoseStamped pose_EE = move_group_interface.getCurrentPose();
    // std::cout << "(" << pose_EE.pose.position.x << "," << pose_EE.pose.position.y << "," << pose_EE.pose.position.z << ")" << std::endl;

    gazebo_model_state_pub = node_handle.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);

    ros::Rate loop(100);

    while(ros::ok())
    {
        geometry_msgs::PoseStamped pose_EE = move_group_interface.getCurrentPose();

        geometry_msgs::Pose pose;
        pose.position.x = pose_EE.pose.position.x + offset.position.x;
        pose.position.y = pose_EE.pose.position.y + offset.position.y;
        pose.position.z = pose_EE.pose.position.z + offset.position.z;
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