#include "sim_controller.h"

class Cube
{
public:
    Cube(std::string color, std::string side, std::string name)
    {
        m_color = color;
        m_side = side;
        m_name = name;
        m_on_table = true;
    }
    std::string getColor()
    {
        return m_color;
    }
    std::string getSide()
    {
        return m_side;
    }
    std::string getName()
    {
        return m_name;
    }
    bool getOnTable()
    {
        return m_on_table;
    }
    void setOnTable(bool v)
    {
        m_on_table = v;
    }

private:
    std::string m_color;
    std::string m_side;
    std::string m_name;
    bool m_on_table;
};
std::vector<Cube> g_cubes;

bool waiting_step_start = true;
bool action_received[2] = {false, false};
bool action_done[2] = {false, false};
std::string g_holding[2] = {"", ""};
ros::ServiceClient move_arm_pose_client[2];
ros::ServiceClient move_arm_named_client[2];
ros::ServiceClient attach_obj_client[2];
ros::ServiceClient attach_plg_client[2];
ros::ServiceClient detach_plg_client[2];
ros::ServiceClient get_model_state_client[2];
ros::ServiceClient set_model_state_client[2];
ros::ServiceClient attach_reset_client[2];
ros::ServiceClient get_world_properties;
ros::ServiceClient move_hand_pass_signal_client;
ros::Publisher r_home_pub;
ros::Publisher h_home_pub;
ros::Publisher visual_signals_pub[2];
ros::Publisher event_log_pub[2];
ros::Publisher text_pluging_pub;
ros::Publisher head_cmd_pub;

// Stack domain
std::map<std::string, geometry_msgs::Pose> locations =
    {
        {"l1",  make_pose(make_point(0.86, 0.24, 0.75), make_quaternion())},
        {"l2",  make_pose(make_point(0.86, 0.44, 0.75), make_quaternion())},
        {"l3",  make_pose(make_point(0.86, 0.34, 0.85), make_quaternion())},
        {"l4",  make_pose(make_point(0.86, 0.24, 0.95), make_quaternion())},
        {"l5",  make_pose(make_point(0.86, 0.44, 0.95), make_quaternion())},
        {"box", make_pose(make_point(0.5, -0.57, 0.80), make_quaternion())},
};

std::map<std::string, geometry_msgs::Pose> init_poses =
    {
        {"box",             make_pose(make_point(0.5, -0.57, 0.7),          make_quaternion())},
        {"box_lid",         make_pose(make_point(0.5, -0.57, 0.7),          make_quaternion())},
        {"cube_b",          make_pose(make_point(1.2141, -0.496866, 0.75),  make_quaternion())},
        {"cube_b_R",        make_pose(make_point(0.5, -0.57, 0.75),         make_quaternion())},
        {"cube_p",          make_pose(make_point(1.22, -0.19, 0.75),        make_quaternion())},
        {"cube_r",          make_pose(make_point(1.20994, -0.674646, 0.75), make_quaternion())},
        {"cube_r_R",        make_pose(make_point(0.5, -0.34, 0.75),         make_quaternion())},
        {"cube_w",          make_pose(make_point(0.5, -0.14, 0.75),         make_quaternion())},
        {"cube_y",          make_pose(make_point(0.86, -0.38, 0.75),        make_quaternion())},
        {"table_slot",      make_pose(make_point(0.86, 0.24, 0.7),          make_quaternion())},
        {"table_slot_0",    make_pose(make_point(0.86, 0.44, 0.7),          make_quaternion())},
        {"robot_head",      make_pose(make_point(0.254, -0.465, 1.07),      make_quaternion())},
};

geometry_msgs::Pose init_human_hand_pose = make_pose(make_point(1.48, 0.5, 0.87), make_quaternion_RPY(0, 0.0, 3.14159));

void init_cubes()
{
    g_cubes.push_back(Cube("r", "R", "cube_r_R"));
    g_cubes.push_back(Cube("r", "H", "cube_r"));
    g_cubes.push_back(Cube("y", "C", "cube_y"));
    g_cubes.push_back(Cube("p", "H", "cube_p"));
    g_cubes.push_back(Cube("b", "H", "cube_b"));
    g_cubes.push_back(Cube("b", "R", "cube_b_R"));
    g_cubes.push_back(Cube("w", "R", "cube_w"));
}

const double tolerance = 0.01;
const double z_offset_grasp = 0.05;

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

geometry_msgs::Quaternion make_quaternion_RPY(double r /*= 0.0*/, double p /*= 0.0*/, double y /*= 0.0*/)
{
    tf2::Quaternion my_q;
    my_q.setRPY(r, p, y);

    geometry_msgs::Quaternion q;
    q.x = my_q.getX();
    q.y = my_q.getY();
    q.z = my_q.getZ();
    q.w = my_q.getW();
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

bool isRobot(AGENT agent)
{
    return agent == AGENT::ROBOT;
}

// ************************* HIGH LEVEL ACTIONS *************************** //

void Pick(AGENT agent, const std::string &color, const std::string &side)
{
    ROS_INFO("\t%s PICK START", get_agent_str(agent).c_str());

    /* FIND OBJ NAME & UPDATE CUBES */
    std::string obj_name;
    for (unsigned int i = 0; i < g_cubes.size(); i++)
    {
        if (g_cubes[i].getOnTable() && g_cubes[i].getSide() == side && g_cubes[i].getColor() == color)
        {
            obj_name = g_cubes[i].getName();
            g_cubes[i].setOnTable(false);
            g_holding[agent] = obj_name;
            break;
        }
    }
    if (obj_name == "")
        throw ros::Exception(agent + " PICK Obj_name for " + color + " " + side + " not found...");

    /* GET OBJ POSE */
    gazebo_msgs::GetModelState srv;
    srv.request.model_name = obj_name;
    if (!get_model_state_client[agent].call(srv) || !srv.response.success)
        throw ros::Exception("Calling service get_model_state failed...");
    geometry_msgs::Pose obj_pose = srv.response.pose;
    show_pose(obj_pose);

    /* MOVE ROBOT HEAD */
    robot_head_follow_obj(agent, obj_name);

    /* MOVE ARM TO OBJ */
    move_pose_target(agent, obj_pose);

    /* GRAB OBJ */
    grab_obj(agent, obj_name);

    /* MOVE ROBOT HEAD */
    robot_head_follow_human(agent);

    /* HOME POSITION */
    move_home(agent);
    ROS_INFO("\t%s PICK END", get_agent_str(agent).c_str());
}

void PlacePose(AGENT agent, geometry_msgs::Pose pose)
{
    ROS_INFO("\t%s PLACE_POSE START", get_agent_str(agent).c_str());

    /* MOVE ROBOT HEAD */
    robot_head_follow_stack(agent);

    /* MOVE ARM TO POSE */
    move_pose_target(agent, pose);

    /* DROP OBJ */
    drop(agent, g_holding[agent]);
    adjust_obj_pose(agent, g_holding[agent], pose);

    /* MOVE ROBOT HEAD */
    robot_head_follow_human(agent);

    /* HOME POSITION */
    move_home(agent);
    ROS_INFO("\t%s PLACE_POSE END", get_agent_str(agent).c_str());
}

void PlaceLocation(AGENT agent, const std::string &location)
{
    PlacePose(agent, locations[location]);
}

void BePassive(AGENT agent)
{
    if (isRobot(agent))
    {
        sim_msgs::Signal sgl;
        sgl.type = sim_msgs::Signal::R_PASS;
        visual_signals_pub[agent].publish(sgl);
    }
    else
    {
        // Hand gesture
        std_srvs::Empty srv;
        move_hand_pass_signal_client.call(srv);
        
        // Visual signal
        sim_msgs::Signal sgl;
        sgl.type = sim_msgs::Signal::H_PASS;
        visual_signals_pub[agent].publish(sgl);
    }
}

void Pushing(AGENT agent)
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
    drop(agent, "cube_r");

    // move home
    move_home(agent);
}

void OpenBox(AGENT agent)
{
    robot_head_follow_obj(agent, "box_lid");

    // Move to box
    move_pose_target(agent, locations["box"]);

    // Open box lid (move to hidden place)
    gazebo_msgs::SetModelState srv_set;
    tf2::Quaternion myQuaternion;
    geometry_msgs::Point point;
    srv_set.request.model_state.model_name = "box_lid";
    point.z = -1.0;
    srv_set.request.model_state.pose.position = point;
    srv_set.request.model_state.pose.orientation = tf2::toMsg(myQuaternion);
    set_model_state_client[agent].call(srv_set);

    robot_head_follow_human(agent);

    // Move home
    move_home(agent);
}

void DropCube(AGENT agent)
{
    std::string obj_name = g_holding[agent];

    robot_head_follow_pose(agent, init_poses[obj_name].position);

    move_pose_target(agent, init_poses[obj_name]);

    drop(agent, obj_name);
    adjust_obj_pose(agent, obj_name, init_poses[obj_name]);

    /* UPDATE CUBES */
    for (unsigned int i = 0; i < g_cubes.size(); i++)
    {
        if (g_cubes[i].getName() == obj_name)
        {
            g_cubes[i].setOnTable(true);
            break;
        }
    }

    robot_head_follow_human(agent);

    move_home(agent);
}

// ************************************************************************ //

// ************************* LOW LEVEL ACTIONS **************************** //

void move_pose_target(AGENT agent, const geometry_msgs::Pose &pose_target, bool human_home)
{
    ROS_INFO("\t\t%s MOVE_POSE_TARGET START", get_agent_str(agent).c_str());
    sim_msgs::MoveArm srv;
    srv.request.pose_target = pose_target;

    /* OFFSET POSE if human and not home */
    if (!isRobot(agent) && !human_home)
        srv.request.pose_target.position.z += z_offset_grasp;

    if (!move_arm_pose_client[agent].call(srv) || !srv.response.success)
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

void move_home(AGENT agent)
{
    if (agent == AGENT::ROBOT)
        move_named_target(agent, "home");
    else if (agent == AGENT::HUMAN)
        move_pose_target(agent, init_human_hand_pose, true);
}

void move_named_target(AGENT agent, const std::string &named_target)
{
    ROS_INFO("\t\t%s MOVE_NAMED_TARGET START", get_agent_str(agent).c_str());
    sim_msgs::MoveArm srv;
    srv.request.named_target = named_target;
    if (!move_arm_named_client[agent].call(srv) || !srv.response.success)
        throw ros::Exception("Calling service move_arm_named_target failed...");
    ROS_INFO("\t\t%s MOVE_NAMED_TARGET END", get_agent_str(agent).c_str());
}

void grab_obj(AGENT agent, const std::string &object)
{
    ROS_INFO("\t\t%s GRAB_OBJ START", get_agent_str(agent).c_str());

    ros::Duration(0.5).sleep();

    gazebo_ros_link_attacher::Attach srv;
    if (agent == AGENT::ROBOT)
    {
        srv.request.model_name_1 = ROBOT_ATTACH_MODEL_NAME;
        srv.request.link_name_1 = ROBOT_ATTACH_LINK_NAME;
    }
    else if (agent == AGENT::HUMAN)
    {
        srv.request.model_name_1 = HUMAN_ATTACH_MODEL_NAME;
        srv.request.link_name_1 = HUMAN_ATTACH_LINK_NAME;
    }
    srv.request.model_name_2 = object;
    srv.request.link_name_2 = "link";
    if (!attach_plg_client[agent].call(srv) || !srv.response.ok)
        throw ros::Exception("Calling service attach_plg_client failed...");

    ROS_INFO("\t\t%s GRAB_OBJ END", get_agent_str(agent).c_str());
}

void drop(AGENT agent, const std::string &object)
{
    ROS_INFO("\t\t%s DROP START", get_agent_str(agent).c_str());

    ros::Duration(0.5).sleep();

    gazebo_ros_link_attacher::Attach srv;
    if (agent == AGENT::ROBOT)
    {
        srv.request.model_name_1 = ROBOT_ATTACH_MODEL_NAME;
        srv.request.link_name_1 = ROBOT_ATTACH_LINK_NAME;
    }
    else if (agent == AGENT::HUMAN)
    {
        srv.request.model_name_1 = HUMAN_ATTACH_MODEL_NAME;
        srv.request.link_name_1 = HUMAN_ATTACH_LINK_NAME;
    }
    srv.request.model_name_2 = object;
    srv.request.link_name_2 = "link";
    std::cout << agent << srv.request.model_name_1 << srv.request.link_name_1 << srv.request.model_name_2 << srv.request.link_name_2 << std::endl;
    if (!detach_plg_client[agent].call(srv) || !srv.response.ok)
        throw ros::Exception("Calling service detach_plg_client failed...");

    // set_obj_rpy(agent, object, 0, 0, 0);

    ROS_INFO("\t\t%s DROP END", get_agent_str(agent).c_str());
}

void set_obj_rpy(AGENT agent, std::string obj_name, float r, float p, float y)
{
    gazebo_msgs::SetModelState srv_set;
    srv_set.request.model_state.model_name = obj_name;

    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(r, p, y);

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

void adjust_obj_pose(AGENT agent, std::string obj_name, geometry_msgs::Pose pose)
{
    set_obj_rpy(agent, obj_name, 0, 0, 0);
    set_obj_pose(agent, obj_name, pose);
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

void robot_head_follow_pose(AGENT agent, geometry_msgs::Point pose)
{
    if(isRobot(agent))
    {
        sim_msgs::HeadCmd msg;
        msg.type = sim_msgs::HeadCmd::FOLLOW_POSE;
        msg.pose = pose; 
        head_cmd_pub.publish(msg);
    }
}

void robot_head_follow_obj(AGENT agent, std::string obj_name)
{
    if(isRobot(agent))
    {
        sim_msgs::HeadCmd msg;
        msg.type = sim_msgs::HeadCmd::FOLLOW_OBJ;
        msg.obj_name = obj_name; 
        head_cmd_pub.publish(msg);
    }
}

void robot_head_follow_human(AGENT agent)
{
    if(isRobot(agent))
    {
        sim_msgs::HeadCmd msg;
        msg.type = sim_msgs::HeadCmd::FOLLOW_HUMAN;
        head_cmd_pub.publish(msg);
    }
}

void robot_head_follow_stack(AGENT agent)
{
    if(isRobot(agent))
    {
        sim_msgs::HeadCmd msg;
        msg.type = sim_msgs::HeadCmd::FOLLOW_STACK;
        head_cmd_pub.publish(msg);
    }
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
        if (waiting_step_start)
        {
            waiting_step_start = false;
            std::cout << std::endl;
            ROS_INFO("=> STEP START");
        }

        std_msgs::String str_msg;
        ROS_INFO("%d type=%d", agent, action.type);

        if (sim_msgs::Action::PASSIVE == action.type)
            BePassive(agent);
        else
        {
            // log event
            sim_msgs::EventLog event;
            event.timestamp = ros::Time::now().toSec();
            event.name = compute_event_name(agent, action, true);
            event_log_pub[agent].publish(event);
            ROS_INFO("Start EVENT SENT");

            action_received[agent] = true;
            std::thread t1(send_visual_signal_action_start, agent, action);
            switch (action.type)
            {
            case sim_msgs::Action::PICK_OBJ:
                if (action.color == "")
                    ROS_ERROR("%d Missing color in action msg!", agent);
                else if (action.side == "")
                    ROS_ERROR("%d Missing side in action msg!", agent);
                else
                    Pick(agent, action.color, action.side);
                break;
            case sim_msgs::Action::PLACE_OBJ:
                if (action.location == "")
                    ROS_ERROR("%d Missing location in action msg!", agent);
                else
                    PlaceLocation(agent, action.location);
                break;
            case sim_msgs::Action::PUSH:
                Pushing(agent);
                break;
            case sim_msgs::Action::OPEN_BOX:
                OpenBox(agent);
                break;
            case sim_msgs::Action::DROP:
                DropCube(agent);
                break;
            default:
                throw ros::Exception("Action type unknown...");
                break;
            }
            t1.join();
            send_visual_signal_action_over(agent, action);
            action_done[agent] = true;
        }
    }
}

void r_home_cb(std_msgs::Empty msg)
{
    move_home(AGENT::ROBOT);
}

void h_home_cb(std_msgs::Empty msg)
{
    move_home(AGENT::HUMAN);
}

bool g_h_start_moving = false;
void h_start_moving_cb(std_msgs::Empty msg)
{
    g_h_start_moving = true;
}
bool g_r_start_moving = false;
void r_start_moving_cb(std_msgs::Empty msg)
{
    g_r_start_moving = true;
}

// ************************************************************************ //

std::string compute_event_name(AGENT agent, sim_msgs::Action action, bool start)
{
    std::string name;

    if (start)
        name = "S_";
    else
        name = "E_";

    if (isRobot(agent))
        name += "RA_";
    else
        name += "HA_";

    switch (action.type)
    {
    case sim_msgs::Action::PICK_OBJ:
        name += "Pick(" + action.color + "," + action.side + ")";
        break;
    case sim_msgs::Action::PLACE_OBJ:
        name += "Place(" + action.color + "," + action.location + ")";
        break;
    case sim_msgs::Action::PUSH:
        name += "Push()";
        break;
    case sim_msgs::Action::OPEN_BOX:
        name += "OpenBox()";
        break;
    case sim_msgs::Action::DROP:
        name += "DropCube(" + action.color + ")";
        break;
    }

    return name;
}

void send_visual_signal_action_start(AGENT agent, sim_msgs::Action action)
{
    ROS_INFO("Waiting for first mvt .........");

    // wait to receive the first "start_moving" message from move_arm or move_hand
    ros::Rate loop(20);
    g_r_start_moving = false;
    g_h_start_moving = false;
    while (ros::ok())
    {
        ros::spinOnce();
        if (isRobot(agent) && g_r_start_moving)
        {
            g_r_start_moving = false;
            break;
        }
        if (!isRobot(agent) && g_h_start_moving)
        {
            g_h_start_moving = false;
            break;
        }
        loop.sleep();
    }
    ROS_INFO("Agent started to move !!!!");

    // send visual signal
    sim_msgs::Signal sgl;
    sgl.id = action.id;
    if (isRobot(agent))
        sgl.type = sim_msgs::Signal::S_RA;
    else
        sgl.type = sim_msgs::Signal::S_HA;
    visual_signals_pub[agent].publish(sgl);
    ROS_INFO("Start signal SENT");
}

void send_visual_signal_action_over(AGENT agent, sim_msgs::Action action)
{
    // send visual signal
    sim_msgs::Signal sgl;
    sgl.id = action.id;
    if (isRobot(agent))
        sgl.type = sim_msgs::Signal::E_RA;
    else
        sgl.type = sim_msgs::Signal::E_HA;
    visual_signals_pub[agent].publish(sgl);
}

bool reset_world_server(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    // Home agents
    home_agents();
    action_received[AGENT::ROBOT] = false;
    action_received[AGENT::HUMAN] = false;
    action_done[AGENT::ROBOT] = false;
    action_done[AGENT::HUMAN] = false;

    // Reset g_cubes
    for (unsigned int i = 0; i < g_cubes.size(); i++)
        g_cubes[i].setOnTable(true);

    // Get world models
    gazebo_msgs::GetWorldProperties srv;
    if (!get_world_properties.call(srv) || !srv.response.success)
        throw ros::Exception("Calling get_world_properties failed...");

    // Detach and reset pose of world models
    gazebo_ros_link_attacher::Attach srv_attach;
    gazebo_msgs::SetModelState srv_set;
    for (std::vector<std::string>::iterator it = srv.response.model_names.begin(); it != srv.response.model_names.end(); it++)
    {
        if ("scene" != (*it) && "human_body" != (*it) && "human_hand" != (*it) && "panda1" != (*it) && "tiago" != (*it))
        {
            // std::cout << (*it) << std::endl;
            srv_attach.request.model_name_2 = (*it);
            srv_attach.request.link_name_2 = "link";
            srv_attach.request.model_name_1 = ROBOT_ATTACH_MODEL_NAME;
            srv_attach.request.link_name_1 = ROBOT_ATTACH_LINK_NAME;
            detach_plg_client[AGENT::ROBOT].call(srv_attach);
            srv_attach.request.model_name_1 = HUMAN_ATTACH_MODEL_NAME;
            srv_attach.request.link_name_1 = HUMAN_ATTACH_LINK_NAME;
            detach_plg_client[AGENT::HUMAN].call(srv_attach);
            srv_set.request.model_state.model_name = (*it);
            srv_set.request.model_state.pose = init_poses[(*it)];
            set_model_state_client[AGENT::ROBOT].call(srv_set);
        }
    }

    std_msgs::String msg;
    msg.data = "Robot Status\nText prompt";
    text_pluging_pub.publish(msg);

    ROS_INFO("World reset ok");

    return true;
}

bool go_idle_pose_server(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    move_named_target(AGENT::ROBOT, "idle");
    return true;
}

void go_idle_pose_cb(const std_msgs::Empty &msg)
{
    move_named_target(AGENT::ROBOT, "idle");
}

bool go_home_pose_server(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    move_named_target(AGENT::ROBOT, "home");
    return true;
}

void go_home_pose_cb(const std_msgs::Empty &msg)
{
    move_named_target(AGENT::ROBOT, "home");
}

void home_agents()
{
    // std_msgs::Empty msg;
    // r_home_pub.publish(msg);
    // move_home(AGENT::HUMAN);

    std_msgs::Empty msg;
    h_home_pub.publish(msg);
    move_home(AGENT::ROBOT);
}

int main(int argc, char **argv)
{
    init_cubes();

    ros::init(argc, argv, "sim_controller");
    ros::NodeHandle node_handle;

    ros::Subscriber robot_action = node_handle.subscribe("/robot_action", 1, robot_action_cb);
    ros::Subscriber human_action = node_handle.subscribe("/human_action", 1, human_action_cb);

    ros::Subscriber r_home = node_handle.subscribe("/r_home", 1, r_home_cb);
    r_home_pub = node_handle.advertise<std_msgs::Empty>("/r_home", 1);

    ros::Subscriber h_home = node_handle.subscribe("/h_home", 1, h_home_cb);
    h_home_pub = node_handle.advertise<std_msgs::Empty>("/h_home", 1);

    const std::string r_move_pose_srv_name("/move_pose_target");
    const std::string r_move_named_topic_name("/move_named_target");
    const std::string r_head_cmd_topic_name("/test_tiago_head");
    const std::string r_head_cmd_ready_name("/tiago_head_ready");
    const std::string h_move_pose_srv_name("/human_hand/move_hand_pose_target");
    const std::string h_pass_topic_name("/human_hand/move_hand_pass_signal");

    move_arm_pose_client[AGENT::ROBOT] = node_handle.serviceClient<sim_msgs::MoveArm>(r_move_pose_srv_name);
    move_arm_pose_client[AGENT::HUMAN] = node_handle.serviceClient<sim_msgs::MoveArm>(h_move_pose_srv_name);
    move_arm_named_client[AGENT::ROBOT] = node_handle.serviceClient<sim_msgs::MoveArm>(r_move_named_topic_name);
    move_hand_pass_signal_client = node_handle.serviceClient<std_srvs::Empty>(h_pass_topic_name);

    attach_plg_client[AGENT::ROBOT] = node_handle.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
    attach_plg_client[AGENT::HUMAN] = node_handle.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
    detach_plg_client[AGENT::ROBOT] = node_handle.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
    detach_plg_client[AGENT::HUMAN] = node_handle.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");

    ros::ServiceServer reset_world_service = node_handle.advertiseService("reset_world", reset_world_server);
    get_world_properties = node_handle.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");

    // ros::Subscriber go_idle_pose_sub = node_handle.subscribe("/go_idle_pose", 1, go_idle_pose_cb);
    // ros::Subscriber go_home_pose_sub = node_handle.subscribe("/go_home_pose", 1, go_home_pose_cb);
    ros::ServiceServer go_idle_pose_service = node_handle.advertiseService("go_idle_pose", go_idle_pose_server);
    ros::ServiceServer go_home_pose_service = node_handle.advertiseService("go_home_pose", go_home_pose_server);

    event_log_pub[AGENT::ROBOT] = node_handle.advertise<sim_msgs::EventLog>("/event_log", 10);
    event_log_pub[AGENT::HUMAN] = node_handle.advertise<sim_msgs::EventLog>("/event_log", 10);
    visual_signals_pub[AGENT::ROBOT] = node_handle.advertise<sim_msgs::Signal>("/robot_visual_signals", 10);
    visual_signals_pub[AGENT::HUMAN] = node_handle.advertise<sim_msgs::Signal>("/human_visual_signals", 10);

    ros::Publisher step_over_pub = node_handle.advertise<std_msgs::Empty>("/step_over", 10);
    std_msgs::Empty empty_msg;

    get_model_state_client[AGENT::ROBOT] = node_handle.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    get_model_state_client[AGENT::HUMAN] = node_handle.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    set_model_state_client[AGENT::ROBOT] = node_handle.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    set_model_state_client[AGENT::HUMAN] = node_handle.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    head_cmd_pub = node_handle.advertise<sim_msgs::HeadCmd>(r_head_cmd_topic_name, 10);

    ros::ServiceClient gazebo_start_client = node_handle.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");

    ros::Subscriber r_start_moving_sub = node_handle.subscribe("/r_start_moving", 1, r_start_moving_cb);
    ros::Subscriber h_start_moving_sub = node_handle.subscribe("/h_start_moving", 1, h_start_moving_cb);

    text_pluging_pub = node_handle.advertise<std_msgs::String>("/text_gazebo_label", 1);

    ros::AsyncSpinner spinner(10);
    spinner.start();

    std_srvs::Empty empty_srv;
    ros::service::waitForService("/gazebo/unpause_physics");
    gazebo_start_client.call(empty_srv);

    ros::service::waitForService(r_move_pose_srv_name);
    ros::service::waitForService(h_move_pose_srv_name);
    ros::service::waitForService(r_head_cmd_ready_name);

    home_agents();

    ROS_INFO("Controllers Ready!");

    ros::Rate loop(50);
    while (ros::ok())
    {
        // ROS_INFO("%d-%d %d-%d", action_received[AGENT::ROBOT], action_done[AGENT::ROBOT], action_received[AGENT::HUMAN], action_done[AGENT::HUMAN]);
        if ((action_received[AGENT::ROBOT] && action_done[AGENT::ROBOT] && action_received[AGENT::HUMAN] && action_done[AGENT::HUMAN]) || (action_received[AGENT::ROBOT] && action_done[AGENT::ROBOT] && !action_received[AGENT::HUMAN] && !action_done[AGENT::HUMAN]) || (!action_received[AGENT::ROBOT] && !action_done[AGENT::ROBOT] && action_received[AGENT::HUMAN] && action_done[AGENT::HUMAN]))
        {
            ROS_INFO("=> STEP OVER");
            // move_named_target(AGENT::ROBOT, "home"); // Reset robot
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