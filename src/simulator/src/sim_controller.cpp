#include "sim_controller.h"

bool waiting_step_start = true;
bool action_received[2] = {false, false};
bool action_done[2] = {false, false};
ros::ServiceClient move_arm_pose_client[2];
ros::ServiceClient move_arm_named_client[2];
ros::ServiceClient attach_obj_client[2];
ros::ServiceClient get_model_state_client[2];
ros::ServiceClient set_model_state_client[2];
ros::ServiceClient attach_reset_client[2];

// world2
// std::map<std::string, geometry_msgs::Pose> locations =
//     {
//         {"loc1", make_pose(make_point(0.62, 0.30, 0.41), make_quaternion())},
//         {"loc2", make_pose(make_point(0.60, -0.29, 0.42), make_quaternion())},
//         {"handover", make_pose(make_point(0.75, 0.0, 0.7), make_quaternion())},
//         {"loc_1", make_pose(make_point(0.75, -0.08, 0.38), make_quaternion())},
//         {"loc_2", make_pose(make_point(0.75, 0.15, 0.38), make_quaternion())},
//         {"loc_3", make_pose(make_point(0.75, 0.38, 0.38), make_quaternion())},
//         {"loc_3above", make_pose(make_point(0.65, 0.38, 0.38), make_quaternion())},
//         {"loc_3below", make_pose(make_point(0.85, 0.38, 0.38), make_quaternion())},
// };

// std::map<std::string, geometry_msgs::Pose> init_poses =
//     {
//         {"cube_r", make_pose(make_point(0.75, -0.42, 0.40), make_quaternion())},
//         {"cube_g", make_pose(make_point(0.62, -0.33, 0.40), make_quaternion())},
//         {"cube_b", make_pose(make_point(0.88, -0.33, 0.40), make_quaternion())},
// };

// Stack_new
std::map<std::string, geometry_msgs::Pose> locations =
    {
        {"l1", make_pose(make_point(0.75, 0.24, 0.37), make_quaternion())},
        {"l2", make_pose(make_point(0.75, 0.38, 0.37), make_quaternion())},
        {"l3", make_pose(make_point(0.75, 0.31, 0.44), make_quaternion())},
        {"l4", make_pose(make_point(0.75, 0.24, 0.51), make_quaternion())},
        {"l5", make_pose(make_point(0.75, 0.38, 0.51), make_quaternion())},
        {"box", make_pose(make_point(0.56, -0.4, 0.45), make_quaternion())},
};

std::map<std::string, geometry_msgs::Pose> init_poses =
    {
        {"cube_r", make_pose(make_point(0.75, -0.22, 0.37), make_quaternion())},
        {"cube_y", make_pose(make_point(0.86, -0.18, 0.37), make_quaternion())},
        {"cube_b", make_pose(make_point(0.88, -0.33, 0.37), make_quaternion())},
        {"cube_p", make_pose(make_point(0.58, -0.32, 0.37), make_quaternion())},
        {"cube_w", make_pose(make_point(0.75, -0.44, 0.37), make_quaternion())},
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

void wait(AGENT agent)
{
    // Be inactive...
    ros::Duration(2.0).sleep();
}

void pushing(AGENT agent)
{
    // move close
    move_location_target(agent, "loc_3above");

    // move arm to obj.pose
    move_obj_target(agent, "cube_r");

    // move obj
    geometry_msgs::Pose delta;
    delta.position.x = 0.2;
    delta_move_obj(agent, "cube_r", delta);

    // drop
    drop(agent);

    // move home
    move_named_target(agent, "home");
}

void open_box(AGENT agent)
{
    move_pose_target(agent, locations["box"]);

    gazebo_msgs::SetModelState srv_set;
    tf2::Quaternion myQuaternion;
    geometry_msgs::Point point;
    srv_set.request.model_state.model_name = "box";
    point.z = -1.0;
    srv_set.request.model_state.pose.position = point;
    srv_set.request.model_state.pose.orientation = tf2::toMsg(myQuaternion);
    set_model_state_client[agent].call(srv_set);

    move_named_target(agent, "home");
}

void drop_cube(AGENT agent)
{
    move_pose_target(agent, init_poses["cube_b"]);

    drop(agent);

    move_named_target(agent, "home");
}


// ************************************************************************ //

// ************************* LOW LEVEL ACTIONS **************************** //

void move_pose_target(AGENT agent, const geometry_msgs::Pose &pose_target)
{
    ROS_INFO("\t\t%s MOVE_POSE_TARGET START", get_agent_str(agent).c_str());
    sim_msgs::MoveArm srv;
    srv.request.pose_target = pose_target;
    if(!move_arm_pose_client[agent].call(srv) || !srv.response.success)
        throw ros::Exception("Calling service move_arm_pose_target failed...");
    ROS_INFO("\t\t%s MOVE_POSE_TARGET END", get_agent_str(agent).c_str());
}

void move_location_target(AGENT agent, const std::string &loc_name)
{
    move_pose_target(agent, locations[loc_name]);
}

void move_obj_target(AGENT agent, const std::string &obj_name)
{
    ROS_INFO("\t\t%s MOVE_OBJ START", get_agent_str(agent).c_str());
    geometry_msgs::Pose obj_pose;
    gazebo_msgs::GetModelState srv;
    srv.request.model_name = obj_name;
    if (!get_model_state_client[agent].call(srv) || !srv.response.success)
        throw ros::Exception("Calling service get_model_state failed...");
    else
        obj_pose = srv.response.pose;
    show_pose(obj_pose);
    move_pose_target(agent, obj_pose);
    ROS_INFO("\t\t%s MOVE_OBJ END", get_agent_str(agent).c_str());
}

void move_named_target(AGENT agent, const std::string &named_target)
{
    ROS_INFO("\t\t%s MOVE_NAMED_TARGET START", get_agent_str(agent).c_str());
    sim_msgs::MoveArm srv;
    srv.request.named_target = named_target;
    if(!move_arm_named_client[agent].call(srv) || !srv.response.success)
        throw ros::Exception("Calling service move_arm_named_target failed...");
    ROS_INFO("\t\t%s MOVE_NAMED_TARGET END", get_agent_str(agent).c_str());
}

void grab_obj(AGENT agent, const std::string &object)
{
    ROS_INFO("\t\t%s GRAB_OBJ START", get_agent_str(agent).c_str());
    sim_msgs::AttachObj srv;
    srv.request.type=sim_msgs::AttachObj::Request::GRAB;
    srv.request.obj_name = object;
    if(!attach_obj_client[agent].call(srv) || !srv.response.success)
        throw ros::Exception("Calling service attach_obj failed...");
    ROS_INFO("\t\t%s GRAB_OBJ END", get_agent_str(agent).c_str());
}

void drop(AGENT agent)
{
    ROS_INFO("\t\t%s DROP START", get_agent_str(agent).c_str());
    sim_msgs::AttachObj srv;
    srv.request.type=sim_msgs::AttachObj::Request::DROP;
    if(!attach_obj_client[agent].call(srv) || !srv.response.success)
        throw ros::Exception("Calling service attach_obj failed...");
    set_obj_rpy(agent, srv.response.obj_dropped_name, 0,0,0);
    ROS_INFO("\t\t%s DROP END", get_agent_str(agent).c_str());
}

void set_obj_rpy(AGENT agent, std::string obj_name, float r, float p, float y)
{
    gazebo_msgs::SetModelState srv_set;
    srv_set.request.model_state.model_name = obj_name;
    
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY( r, p, y );
    
    gazebo_msgs::GetModelState srv_get;
    srv_get.request.model_name = obj_name;
    get_model_state_client[agent].call(srv_get);

    srv_set.request.model_state.pose.position = srv_get.response.pose.position;
    srv_set.request.model_state.pose.orientation = tf2::toMsg(myQuaternion);

    set_model_state_client[agent].call(srv_set);
}

void set_obj_pose(AGENT agent, std::string obj_name, geometry_msgs::Pose pose)
{
    gazebo_msgs::SetModelState srv;
    srv.request.model_state.model_name = obj_name;
    srv.request.model_state.pose = pose;
    set_model_state_client[agent].call(srv);
 }

void delta_move_obj(AGENT agent, std::string obj_name, geometry_msgs::Pose delta_move)
{
    gazebo_msgs::GetModelState srv;
    srv.request.model_name = obj_name;
    get_model_state_client[agent].call(srv);
    geometry_msgs::Pose new_obj_pose;
    new_obj_pose.position.x = srv.response.pose.position.x + delta_move.position.x;
    new_obj_pose.position.y = srv.response.pose.position.y + delta_move.position.y;
    new_obj_pose.position.z = srv.response.pose.position.z + delta_move.position.z;
    set_obj_pose(agent, obj_name, new_obj_pose);
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
        ROS_INFO("%d type=%d", agent, action.type);
        switch (action.type)
        {
        case sim_msgs::Action::PICK_OBJ:
            if (action.obj == "")
                ROS_ERROR("%d Missing object in action msg!", agent);
            else
                pick(agent, action.obj);
            break;
        case sim_msgs::Action::PICK_R:
            pick(agent, "cube_r");
            break;
        case sim_msgs::Action::PICK_G:
            pick(agent, "cube_g");
            break;
        case sim_msgs::Action::PICK_B:
            if(agent==AGENT::ROBOT)
                pick(agent, "cube_b_R");
            else if(agent==AGENT::HUMAN)
                pick(agent, "cube_b");
            break;
        case sim_msgs::Action::PICK_Y:
            pick(agent, "cube_y");
            break;
        case sim_msgs::Action::PICK_P:
            pick(agent, "cube_p");
            break;
        case sim_msgs::Action::PICK_W:
            pick(agent, "cube_w");
            break;
        case sim_msgs::Action::PLACE_OBJ:
            if (action.location == "")
                ROS_ERROR("%d Missing location in action msg!", agent);
            else
                place_location(agent, action.location);
            break;
        case sim_msgs::Action::PLACE_1:
            place_location(agent, "l1");
            break;        
        case sim_msgs::Action::PLACE_2:
            place_location(agent, "l2");
            break;        
        case sim_msgs::Action::PLACE_3:
            place_location(agent, "l3");
            break;        
        case sim_msgs::Action::PLACE_4:
            place_location(agent, "l4");
            break;        
        case sim_msgs::Action::PLACE_5:
            place_location(agent, "l5");
            break;
        case sim_msgs::Action::PUSH:
            pushing(agent);
            break;
        case sim_msgs::Action::OPEN_BOX:
            open_box(agent);
            break;
        case sim_msgs::Action::DROP:
            drop_cube(agent);
            break;
        case sim_msgs::Action::PASSIVE:
            wait(agent);
            break;
        default:
            throw ros::Exception("Action type unknown...");
            break;
        }

        action_done[agent] = true;
    }
}

// ************************************************************************ //

bool reset_obj_server(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    // reset attach
    std_srvs::Empty srv_e;
    attach_reset_client[AGENT::ROBOT].call(srv_e);
    attach_reset_client[AGENT::HUMAN].call(srv_e);

    // set obj to init pose
    gazebo_msgs::SetModelState srv;
    std::vector<std::string> list_obj = {"cube_r", "cube_g", "cube_b"};
    for(std::vector<std::string>::iterator it=list_obj.begin(); it!=list_obj.end(); ++it)
    {
        srv.request.model_state.model_name = *it;
        srv.request.model_state.pose = init_poses[*it];
        set_model_state_client[AGENT::ROBOT].call(srv);
    }
    return true;
}

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

    set_model_state_client[AGENT::ROBOT] = node_handle.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    set_model_state_client[AGENT::HUMAN] = node_handle.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    ros::ServiceServer reset_service = node_handle.advertiseService("reset_obj", reset_obj_server);
    attach_reset_client[AGENT::ROBOT] = node_handle.serviceClient<std_srvs::Empty>("/panda1/attach_reset");
    attach_reset_client[AGENT::HUMAN] = node_handle.serviceClient<std_srvs::Empty>("/panda2/attach_reset");

    ros::ServiceClient gazebo_start_client = node_handle.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");

    ros::AsyncSpinner spinner(4);
    spinner.start();

    std_srvs::Empty empty_srv;
    ros::service::waitForService("/gazebo/unpause_physics");
    gazebo_start_client.call(empty_srv);
    
    ros::service::waitForService("/panda1/move_pose_target");
    ros::service::waitForService("/panda2/move_pose_target");
    ros::service::waitForService("/panda1/move_named_target");
    ros::service::waitForService("/panda2/move_named_target");
    move_named_target(AGENT::ROBOT, "home");
    move_named_target(AGENT::HUMAN, "home");
    // Publish message to parallelize

    ROS_INFO("Controllers Ready!");

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