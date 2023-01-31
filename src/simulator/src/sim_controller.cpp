
#include "sim_controller.h"

bool waiting_step_start = true;
bool action_received[2] = {false,false};
bool action_done[2] = {false,false};
bool moving[2] = {false,false};
ros::Publisher move_arm_pose_pub[2];
ros::Publisher move_arm_named_pub[2];
ros::Publisher attach_obj_pub[2];
moveit::planning_interface::MoveGroupInterface* move_group_interface[2];
ros::ServiceClient get_model_state_client[2];

std::map<std::string,geometry_msgs::Pose> locations=
{
    {"loc1", make_pose(make_point(0.62, 0.30, 0.41), make_quaternion())},
    {"loc2", make_pose(make_point(0.60, -0.29, 0.42), make_quaternion())},
    {"handover", make_pose(make_point(0.75, 0.0, 0.7), make_quaternion())},
};

const double tolerance = 0.01;

// ************************************************************************ //

geometry_msgs::Point make_point(double x, double y, double z) {
  geometry_msgs::Point p;
  p.x = x; p.y = y; p.z = z;
  return p;
}

geometry_msgs::Quaternion make_quaternion(double x /*= 0.0*/, double y /*= 0.0*/, double z /*= 0.0*/, double w /*= 1.0*/) {
  geometry_msgs::Quaternion q;
  q.x = x; q.y = y; q.z = z; q.w = w;
  return q;
}

geometry_msgs::Pose make_pose(geometry_msgs::Point p, geometry_msgs::Quaternion q) {
  geometry_msgs::Pose pose;
  pose.position = p;
  pose.orientation = q;
  return pose;
}

void show_pose(geometry_msgs::Pose pose)
{
    ROS_INFO("\tpose= (%.2f, %.2f, %.2f)(%.2f, %.2f, %.2f, %.2f)", pose.position.x, pose.position.y, pose.position.z, 
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
}

std::string get_agent_str(AGENT agent)
{
    if(agent==AGENT::ROBOT)
        return "ROBOT";
    else if(agent==AGENT::HUMAN)
        return "HUMAN";
    else
        throw ros::Exception("Agent unknown...");
}
// ************************************************************************ //

void check_moving(AGENT agent, geometry_msgs::Pose& new_pose, geometry_msgs::Pose& previous_pose, int& count)
{
    // ROS_INFO("\tcheck pose %d %d %d", agent, moving[agent], count);
    // show_pose(previous_pose);
    // show_pose(new_pose);

    if(abs(new_pose.position.x-previous_pose.position.x)<tolerance
    && abs(new_pose.position.y-previous_pose.position.y)<tolerance
    && abs(new_pose.position.z-previous_pose.position.z)<tolerance
    && abs(new_pose.orientation.w-previous_pose.orientation.w)<tolerance
    && abs(new_pose.orientation.x-previous_pose.orientation.x)<tolerance
    && abs(new_pose.orientation.y-previous_pose.orientation.y)<tolerance
    && abs(new_pose.orientation.z-previous_pose.orientation.z)<tolerance
    )
    {
        if(moving[agent])
        {
        if(count>3)
        {
            moving[agent]=false;
            count=0;
        }
        else
            count++;
        }
    }
    else
    {
        moving[agent] = true;
        count = 0;
    }
    previous_pose = new_pose;
}

void wait_still_moving(AGENT agent)
{
    moving[agent] = true;
    geometry_msgs::Pose previous_pose = move_group_interface[agent]->getCurrentPose().pose;
    int count=0;
    ros::Duration(2).sleep();

    ros::Rate loop(20);
    ROS_INFO("\t\tWaiting still moving...");
    while(ros::ok() && moving[agent])
    {
        geometry_msgs::Pose pose = move_group_interface[agent]->getCurrentPose().pose;
        check_moving(agent, pose, previous_pose, count);
        loop.sleep();
    }
    ROS_INFO("\t\tMotion done!");
}

// ************************* HIGH LEVEL ACTIONS *************************** //

void pick_object(AGENT agent, const std::string& obj)
{
    ROS_INFO("%s PICK_OBJECT START", get_agent_str(agent).c_str());

    /* MOVE ARM TO OBJ */
    geometry_msgs::Pose obj_pose;
    gazebo_msgs::GetModelState srv;
    srv.request.model_name = obj;
    if(!get_model_state_client[agent].call(srv) && srv.response.success)
        throw ros::Exception("Calling service get_model_state failed...");
    else
        obj_pose = srv.response.pose;
    move_pose_target(agent, obj_pose);

    /* GRAB OBJ */
    grab_obj(agent, obj);

    /* MOVE ARM BACK*/
    move_named_target(agent, "home");

    ROS_INFO("%s PICK_OBJECT END", get_agent_str(agent).c_str());
}

void place_object(AGENT agent, const std::string& loc)
{
    ROS_INFO("%s PLACE_OBJ START", get_agent_str(agent).c_str());

    /* MOVE ARM TO place */
    move_pose_target(agent, locations[loc]);

    /* DROP OBJ */
    drop(agent);

    /* MOVE ARM BACK*/
    move_named_target(agent, "home");

    ROS_INFO("%s PLACE_OBJ END", get_agent_str(agent).c_str());
}

// ************************************************************************ //

// ************************* LOW LEVEL ACTIONS **************************** //

void move_pose_target(AGENT agent, const geometry_msgs::Pose& pose_target)
{
    ROS_INFO("\t%s MOVE_POSE_TARGET START", get_agent_str(agent).c_str());
    move_arm_pose_pub[agent].publish(pose_target);
    wait_still_moving(agent);
    ROS_INFO("\t%s MOVE_POSE_TARGET END", get_agent_str(agent).c_str());
}

void move_named_target(AGENT agent, const std::string& named_target)
{
    ROS_INFO("\t%s MOVE_NAMED_TARGET START", get_agent_str(agent).c_str());
    std_msgs::String str_msg;
    str_msg.data = named_target;
    move_arm_named_pub[agent].publish(str_msg);
    wait_still_moving(agent);
    ROS_INFO("\t%s MOVE_NAMED_TARGET START", get_agent_str(agent).c_str());
}

void grab_obj(AGENT agent, const std::string& object)
{
    ROS_INFO("\t%s GRAB_OBJ START", get_agent_str(agent).c_str());
    std_msgs::String str_msg;
    str_msg.data = "grab "+object;
    attach_obj_pub[agent].publish(str_msg);
    ros::Duration(0.2).sleep();
    ROS_INFO("\t%s GRAB_OBJ END", get_agent_str(agent).c_str());
}

void drop(AGENT agent)
{
    ROS_INFO("\t%s DROP START", get_agent_str(agent).c_str());
    std_msgs::String str_msg;
    str_msg.data = "drop";
    attach_obj_pub[agent].publish(str_msg);
    ros::Duration(1).sleep();
    ROS_INFO("\t%s DROP END", get_agent_str(agent).c_str());
}

// ****************************** CALLBACKS ******************************* //

void robot_action_cb(const sim_msgs::Action& msg)
{
    manage_action(AGENT::ROBOT, msg);
}

void human_action_cb(const sim_msgs::Action& msg)
{
    manage_action(AGENT::HUMAN, msg);
}

void manage_action(AGENT agent, const sim_msgs::Action& action)
{
    if(action_received[agent])
        ROS_ERROR("Agent %d is already performing an action... New action skipped.", agent);
    else
    {
        action_received[agent] = true;

        if(waiting_step_start)
        {
            waiting_step_start=false;
            std::cout << std::endl;
            ROS_INFO("=> STEP START");
        }

        std_msgs::String str_msg;
        switch(action.type)
        {
            case sim_msgs::Action::PICK_OBJ:
                if(action.obj=="")
                    ROS_ERROR("Missing object in action msg!");
                else
                    pick_object(agent, action.obj);
                break;
            case sim_msgs::Action::PLACE_OBJ:
                if(action.location=="")
                    ROS_ERROR("Missing location in action msg!");
                else
                    place_object(agent, action.location);
                break;
            default:
                throw ros::Exception("Action type unknown...");
                break;
        }

        action_done[agent] = true;
    }
}

// ************************************************************************ //

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sim_controller");
    ros::NodeHandle node_handle;
    
    ros::Subscriber robot_action = node_handle.subscribe("/robot_action", 1, robot_action_cb);
    ros::Subscriber human_action = node_handle.subscribe("/human_action", 1, human_action_cb);

    move_arm_pose_pub[AGENT::ROBOT] = node_handle.advertise<geometry_msgs::Pose>("/panda1_move_goal_pose", 10);
    move_arm_pose_pub[AGENT::HUMAN] = node_handle.advertise<geometry_msgs::Pose>("/panda2_move_goal_pose", 10);

    move_arm_named_pub[AGENT::ROBOT] = node_handle.advertise<std_msgs::String>("/panda1_move_named_target", 10);
    move_arm_named_pub[AGENT::HUMAN] = node_handle.advertise<std_msgs::String>("/panda2_move_named_target", 10);

    attach_obj_pub[AGENT::ROBOT] = node_handle.advertise<std_msgs::String>("/panda1_attach_cmd", 10);
    attach_obj_pub[AGENT::HUMAN] = node_handle.advertise<std_msgs::String>("/panda2_attach_cmd", 10);


    ros::Publisher step_over_pub = node_handle.advertise<std_msgs::Empty>("/step_over", 10);
    std_msgs::Empty empty_msg;

    move_group_interface[AGENT::ROBOT] = new moveit::planning_interface::MoveGroupInterface("panda1_arm");
    move_group_interface[AGENT::HUMAN] = new moveit::planning_interface::MoveGroupInterface("panda2_arm");

    get_model_state_client[AGENT::ROBOT] = node_handle.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    get_model_state_client[AGENT::HUMAN] = node_handle.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Rate loop(50);
    bool transi_step_over=true;
    while(ros::ok())
    {
        if((action_received[AGENT::ROBOT]  && action_done[AGENT::ROBOT]  && action_received[AGENT::HUMAN]  && action_done[AGENT::HUMAN])
        || (action_received[AGENT::ROBOT]  && action_done[AGENT::ROBOT]  && !action_received[AGENT::HUMAN] && !action_done[AGENT::HUMAN])
        || (!action_received[AGENT::ROBOT] && !action_done[AGENT::ROBOT] && action_received[AGENT::HUMAN]  && action_done[AGENT::HUMAN]))
        {
            ROS_INFO("=> STEP OVER");
            step_over_pub.publish(empty_msg);
            action_received[AGENT::ROBOT] = false;
            action_done[AGENT::ROBOT] = false;
            action_received[AGENT::HUMAN] = false;
            action_done[AGENT::HUMAN] = false;
            waiting_step_start = true;
        }

        loop.sleep();
    }

    delete move_group_interface[AGENT::ROBOT];
    delete move_group_interface[AGENT::HUMAN];

    return 0;
}