
#include "sim_controller.h"

bool waiting_step_start = true;
bool action_received[2] = {false, false};
bool action_done[2] = {false, false};
ros::ServiceClient move_arm_pose_client[2];
ros::ServiceClient move_arm_named_client[2];
ros::ServiceClient attach_obj_client[2];
ros::ServiceClient get_model_state_client[2];
// ros::ServiceClient set_model_state_client[2]; ??????

std::map<std::string, geometry_msgs::Pose> locations =
    {
        {"loc1", make_pose(make_point(0.62, 0.30, 0.41), make_quaternion())},
        {"loc2", make_pose(make_point(0.60, -0.29, 0.42), make_quaternion())},
        {"handover", make_pose(make_point(0.75, 0.0, 0.7), make_quaternion())},
        {"loc_1", make_pose(make_point(0.75, -0.08, 0.41), make_quaternion())},
        {"loc_2", make_pose(make_point(0.75, 0.15, 0.41), make_quaternion())},
        {"loc_3", make_pose(make_point(0.75, 0.38, 0.41), make_quaternion())},
        {"loc_3bis", make_pose(make_point(0.57, 0.38, 0.41), make_quaternion())},
};

const double tolerance = 0.01;

// ************************************************************************ //

geometry_msgs::Point make_point(double x, double y, double z)
{
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

geometry_msgs::Quaternion make_quaternion(double x /*= 0.0*/, double y /*= 0.0*/, double z /*= 0.0*/, double w /*= 1.0*/)
{
    geometry_msgs::Quaternion q;
    q.x = x;
    q.y = y;
    q.z = z;
    q.w = w;
    return q;
}

geometry_msgs::Pose make_pose(geometry_msgs::Point p, geometry_msgs::Quaternion q)
{
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
    if (agent == AGENT::ROBOT)
        return "ROBOT";
    else if (agent == AGENT::HUMAN)
        return "HUMAN";
    else
        throw ros::Exception("Agent unknown...");
}

// ************************* HIGH LEVEL ACTIONS *************************** //

void pick(AGENT agent, const std::string &obj)
{
    ROS_INFO("\t%s PICK START", get_agent_str(agent).c_str());
    
    /* MOVE ARM TO OBJ */
    move_obj_target(agent, obj);

    /* GRAB OBJ */
    grab_obj(agent, obj);

    /* MOVE ARM BACK*/
    move_named_target(agent, "home");
    ROS_INFO("\t%s PICK END", get_agent_str(agent).c_str());
}

void place_pose(AGENT agent, const geometry_msgs::Pose &pose)
{
    ROS_INFO("\t%s PLACE_POSE START", get_agent_str(agent).c_str());
    show_pose(pose);

    move_pose_target(agent, pose);

    drop(agent);

    move_named_target(agent, "home");
    ROS_INFO("\t%s PLACE_POSE END", get_agent_str(agent).c_str());
}

void place_location(AGENT agent, const std::string &location)
{
    place_pose(agent, locations[location]);
}

// ************************************************************************ //

// ************************* LOW LEVEL ACTIONS **************************** //

void move_pose_target(AGENT agent, const geometry_msgs::Pose &pose_target)
{
    ROS_INFO("\t%s MOVE_POSE_TARGET START", get_agent_str(agent).c_str());
    sim_msgs::MoveArm srv;
    srv.request.pose_target = pose_target;
    if(!move_arm_pose_client[agent].call(srv) || !srv.response.success)
        throw ros::Exception("Calling service move_arm_pose_target failed...");
    ROS_INFO("\t%s MOVE_POSE_TARGET END", get_agent_str(agent).c_str());
}

void move_obj_target(AGENT agent, const std::string &obj_name)
{
    geometry_msgs::Pose obj_pose;
    gazebo_msgs::GetModelState srv;
    srv.request.model_name = obj_name;
    if (!get_model_state_client[agent].call(srv) || !srv.response.success)
        throw ros::Exception("Calling service get_model_state failed...");
    else
        obj_pose = srv.response.pose;
    show_pose(obj_pose);
    move_pose_target(agent, obj_pose);
}

void move_named_target(AGENT agent, const std::string &named_target)
{
    ROS_INFO("\t%s MOVE_NAMED_TARGET START", get_agent_str(agent).c_str());
    sim_msgs::MoveArm srv;
    srv.request.named_target = named_target;
    if(!move_arm_named_client[agent].call(srv) || !srv.response.success)
        throw ros::Exception("Calling service move_arm_named_target failed...");
    ROS_INFO("\t%s MOVE_NAMED_TARGET START", get_agent_str(agent).c_str());
}

void grab_obj(AGENT agent, const std::string &object)
{
    ROS_INFO("\t%s GRAB_OBJ START", get_agent_str(agent).c_str());
    sim_msgs::AttachObj srv;
    srv.request.type=sim_msgs::AttachObj::Request::GRAB;
    srv.request.obj_name = object;
    if(!attach_obj_client[agent].call(srv) || !srv.response.success)
        throw ros::Exception("Calling service attach_obj failed...");
    ROS_INFO("\t%s GRAB_OBJ END", get_agent_str(agent).c_str());
}

void drop(AGENT agent)
{
    ROS_INFO("\t%s DROP START", get_agent_str(agent).c_str());
    sim_msgs::AttachObj srv;
    srv.request.type=sim_msgs::AttachObj::Request::DROP;
    if(!attach_obj_client[agent].call(srv) || !srv.response.success)
        throw ros::Exception("Calling service attach_obj failed...");
    ROS_INFO("\t%s DROP END", get_agent_str(agent).c_str());
}

// ****************************** CALLBACKS ******************************* //

void robot_action_cb(const sim_msgs::Action &msg)
{
    manage_action(AGENT::ROBOT, msg);
}

void human_action_cb(const sim_msgs::Action &msg)
{
    manage_action(AGENT::HUMAN, msg);
}

void manage_action(AGENT agent, const sim_msgs::Action &action)
{
    if (action_received[agent])
        ROS_ERROR("Agent %d is already performing an action... New action skipped.", agent);
    else
    {
        action_received[agent] = true;

        if (waiting_step_start)
        {
            waiting_step_start = false;
            std::cout << std::endl;
            ROS_INFO("=> STEP START");
        }

        std_msgs::String str_msg;
        switch (action.type)
        {
        case sim_msgs::Action::PICK_OBJ:
        case sim_msgs::Action::PICK_B:
        case sim_msgs::Action::PICK_R:
        case sim_msgs::Action::PICK_G:
            if (action.obj == "")
                ROS_ERROR("Missing object in action msg!");
            else
                pick(agent, action.obj);
            break;
        case sim_msgs::Action::PLACE_OBJ:
        case sim_msgs::Action::PLACE_1:
        case sim_msgs::Action::PLACE_2:
        case sim_msgs::Action::PLACE_3:
        case sim_msgs::Action::PLACE_4:
            if (action.location == "")
                ROS_ERROR("Missing location in action msg!");
            else
                place_location(agent, action.location);
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

    move_arm_pose_client[AGENT::ROBOT] = node_handle.serviceClient<sim_msgs::MoveArm>("/panda1/move_pose_target");
    move_arm_pose_client[AGENT::HUMAN] = node_handle.serviceClient<sim_msgs::MoveArm>("/panda2/move_pose_target");

    move_arm_named_client[AGENT::ROBOT] = node_handle.serviceClient<sim_msgs::MoveArm>("/panda1/move_named_target");
    move_arm_named_client[AGENT::HUMAN] = node_handle.serviceClient<sim_msgs::MoveArm>("/panda2/move_named_target");


    attach_obj_client[AGENT::ROBOT] = node_handle.serviceClient<sim_msgs::AttachObj>("/panda1/attach_obj");
    attach_obj_client[AGENT::HUMAN] = node_handle.serviceClient<sim_msgs::AttachObj>("/panda2/attach_obj");

    ros::Publisher step_over_pub = node_handle.advertise<std_msgs::Empty>("/step_over", 10);
    std_msgs::Empty empty_msg;

    get_model_state_client[AGENT::ROBOT] = node_handle.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    get_model_state_client[AGENT::HUMAN] = node_handle.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Rate loop(50);
    bool transi_step_over = true;
    while (ros::ok())
    {
        if ((action_received[AGENT::ROBOT] && action_done[AGENT::ROBOT] && action_received[AGENT::HUMAN] && action_done[AGENT::HUMAN]) 
        || (action_received[AGENT::ROBOT] && action_done[AGENT::ROBOT] && !action_received[AGENT::HUMAN] && !action_done[AGENT::HUMAN]) 
        || (!action_received[AGENT::ROBOT] && !action_done[AGENT::ROBOT] && action_received[AGENT::HUMAN] && action_done[AGENT::HUMAN]))
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

    return 0;
}