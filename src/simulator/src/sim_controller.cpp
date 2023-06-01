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
ros::Publisher r_home_pub;

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
    {"box",            make_pose(make_point(0.5, -0.57, 0.7),           make_quaternion(0, -0, 0))},
    {"box_lid",        make_pose(make_point(0.5, -0.57, 0.7),           make_quaternion(0, -0, 0))},
    {"cube_b",         make_pose(make_point(1.2141, -0.496866, 0.75),   make_quaternion(0, -0, 0))},
    {"cube_b_R",       make_pose(make_point(0.5, -0.57, 0.75),          make_quaternion(0, -0, 0))},
    {"cube_p",         make_pose(make_point(1.22, -0.19, 0.75),         make_quaternion(0, -0, 0))},
    {"cube_r",         make_pose(make_point(1.20994, -0.674646, 0.75),  make_quaternion(0, -0, 0))},
    {"cube_r_R",       make_pose(make_point(0.5, -0.34, 0.75),          make_quaternion(0, -0, 0))},
    {"cube_w",         make_pose(make_point(0.5, -0.14, 0.75),          make_quaternion(0, -0, 0))},
    {"cube_y",         make_pose(make_point(0.86, -0.38, 0.75),         make_quaternion(0, -0, 0))},
    {"table_slot",     make_pose(make_point(0.86, 0.24, 0.7),           make_quaternion(0, -0, 0))},
    {"table_slot_0",   make_pose(make_point(0.86, 0.44, 0.7),           make_quaternion(0, -0, 0))},
};

geometry_msgs::Pose init_human_hand_pose = make_pose(make_point(1.38, 0.5, 0.87), make_quaternion_RPY(0, 0, 3.14159));

void init_cubes()
{
    g_cubes.push_back( Cube("r", "R", "cube_r_R") );
    g_cubes.push_back( Cube("r", "H", "cube_r") );
    g_cubes.push_back( Cube("y", "C", "cube_y") );
    g_cubes.push_back( Cube("p", "H", "cube_p") );
    g_cubes.push_back( Cube("b", "H", "cube_b") );
    g_cubes.push_back( Cube("b", "R", "cube_b_R") );
    g_cubes.push_back( Cube("w", "R", "cube_w") );
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
    return agent==AGENT::ROBOT;
}

// ************************* HIGH LEVEL ACTIONS *************************** //

void Pick(AGENT agent, const std::string &color, const std::string &side)
{
    ROS_INFO("\t%s PICK START", get_agent_str(agent).c_str());
    
    /* FIND OBJ NAME & UPDATE CUBES */
    std::string obj_name;
    for(unsigned int i=0; i<g_cubes.size(); i++)
    {
        if(g_cubes[i].getOnTable() && g_cubes[i].getSide()==side && g_cubes[i].getColor()==color)
        {
            obj_name = g_cubes[i].getName();
            g_cubes[i].setOnTable(false);
            g_holding[agent] = obj_name;
            break;
        }
    }
    if(obj_name=="")
        throw ros::Exception("PICK Obj_name for "+color+" "+side+" not found...");

    /* GET OBJ POSE */
    gazebo_msgs::GetModelState srv;
    srv.request.model_name = obj_name;
    if (!get_model_state_client[agent].call(srv) || !srv.response.success)
        throw ros::Exception("Calling service get_model_state failed...");
    geometry_msgs::Pose obj_pose = srv.response.pose;
    show_pose(obj_pose);

    /* MOVE ARM TO OBJ */
    move_pose_target(agent, obj_pose);

    /* GRAB OBJ */
    grab_obj(agent, obj_name);

    /* HOME POSITION */
    move_home(agent);
    ROS_INFO("\t%s PICK END", get_agent_str(agent).c_str());
}

void PlacePose(AGENT agent, geometry_msgs::Pose pose)
{
    ROS_INFO("\t%s PLACE_POSE START", get_agent_str(agent).c_str());

    /* MOVE ARM TO POSE */
    move_pose_target(agent, pose);

    /* DROP OBJ */
    drop(agent, g_holding[agent]);

    /* HOME POSITION */
    move_home(agent);
    ROS_INFO("\t%s PLACE_POSE END", get_agent_str(agent).c_str());
}

void PlaceLocation(AGENT agent, const std::string &location)
{
    PlacePose(agent, locations[location]);
}

void Wait(AGENT agent)
{
    // Be inactive...
    ros::Duration(2.0).sleep();
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

    // Move home
    move_home(agent);
}

void DropCube(AGENT agent)
{
    std::string obj_name = g_holding[agent];
    move_pose_target(agent, init_poses[obj_name]);

    drop(agent, obj_name);

    /* UPDATE CUBES */
    for(unsigned int i=0; i<g_cubes.size(); i++)
    {
        if(g_cubes[i].getName()==obj_name)
        {
            g_cubes[i].setOnTable(true);
            break;
        }
    }

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
    if(!isRobot(agent) && !human_home)
        srv.request.pose_target.position.z += z_offset_grasp;
        
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

void move_home(AGENT agent)
{
    if(agent==AGENT::ROBOT)
        move_named_target(agent, "home");
    else if(agent==AGENT::HUMAN)
        move_pose_target(agent, init_human_hand_pose, true);
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

    gazebo_ros_link_attacher::Attach srv;
    if(agent==AGENT::ROBOT)
    {
        srv.request.model_name_1 = ROBOT_ATTACH_MODEL_NAME;
        srv.request.link_name_1 = ROBOT_ATTACH_LINK_NAME;
    }
    else if(agent==AGENT::HUMAN)
    {
        srv.request.model_name_1 = HUMAN_ATTACH_MODEL_NAME;
        srv.request.link_name_1 = HUMAN_ATTACH_LINK_NAME;
    }
    srv.request.model_name_2 = object;
    srv.request.link_name_2 = "link";
    if(!attach_plg_client[agent].call(srv) || !srv.response.ok)
        throw ros::Exception("Calling service attach_plg_client failed...");

    ROS_INFO("\t\t%s GRAB_OBJ END", get_agent_str(agent).c_str());
}

void drop(AGENT agent, const std::string &object)
{
    ROS_INFO("\t\t%s DROP START", get_agent_str(agent).c_str());

    gazebo_ros_link_attacher::Attach srv;
    if(agent==AGENT::ROBOT)
    {
        srv.request.model_name_1 = ROBOT_ATTACH_MODEL_NAME;
        srv.request.link_name_1 = ROBOT_ATTACH_LINK_NAME;
    }
    else if(agent==AGENT::HUMAN)
    {
        srv.request.model_name_1 = HUMAN_ATTACH_MODEL_NAME;
        srv.request.link_name_1 = HUMAN_ATTACH_LINK_NAME;
    }
    srv.request.model_name_2 = object;
    srv.request.link_name_2 = "link";
    std::cout << agent << srv.request.model_name_1 << srv.request.link_name_1 << srv.request.model_name_2 << srv.request.link_name_2 << std::endl;
    if(!detach_plg_client[agent].call(srv) || !srv.response.ok)
        throw ros::Exception("Calling service detach_plg_client failed...");

    set_obj_rpy(agent, object, 0,0,0);

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
            if (action.color=="")
                ROS_ERROR("%d Missing color in action msg!", agent);
            else if(action.side=="")
                ROS_ERROR("%d Missing side in action msg!", agent);
            else
                Pick(agent, action.color, action.side);
            break;
        case sim_msgs::Action::PLACE_OBJ:
            if (action.location=="")
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
        case sim_msgs::Action::PASSIVE:
            Wait(agent);
            break;
        default:
            throw ros::Exception("Action type unknown...");
            break;
        }

        action_done[agent] = true;
    }
}

void r_home_cb(std_msgs::Empty msg)
{
    move_home(AGENT::ROBOT);
}

// ************************************************************************ //

bool reset_world_server(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    // Home agents
    home_agents();

    // Reset g_cubes
    for(unsigned int i=0; i<g_cubes.size(); i++)
        g_cubes[i].setOnTable(true);

    // Get world models
    gazebo_msgs::GetWorldProperties srv;
    if(!get_world_properties.call(srv) || !srv.response.success)
        throw ros::Exception("Calling get_world_properties failed...");

    // Detach and reset pose of world models
    gazebo_ros_link_attacher::Attach srv_attach;
    gazebo_msgs::SetModelState srv_set;
    for(std::vector<std::string>::iterator it=srv.response.model_names.begin(); it!=srv.response.model_names.end(); it++)
    {
        if("scene"      != (*it)
        && "human_body" != (*it)
        && "human_hand" != (*it)
        && "panda1"     != (*it)
        )
        {
            // std::cout << (*it) << std::endl;
            srv_attach.request.model_name_2 = (*it);
            srv_attach.request.link_name_2  = "link";
            srv_attach.request.model_name_1 = ROBOT_ATTACH_MODEL_NAME;
            srv_attach.request.link_name_1  = ROBOT_ATTACH_LINK_NAME;
            detach_plg_client[AGENT::ROBOT].call(srv_attach);
            srv_attach.request.model_name_1 = HUMAN_ATTACH_MODEL_NAME;
            srv_attach.request.link_name_1  = HUMAN_ATTACH_LINK_NAME;
            detach_plg_client[AGENT::HUMAN].call(srv_attach);
            srv_set.request.model_state.model_name = (*it);
            srv_set.request.model_state.pose = init_poses[(*it)];
            set_model_state_client[AGENT::ROBOT].call(srv_set);
        }
    }

    ROS_INFO("World reset ok");

    return true;
}

void home_agents()
{
    std_msgs::Empty msg;
    r_home_pub.publish(msg);
    move_home(AGENT::HUMAN);
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

    move_arm_pose_client[AGENT::ROBOT] = node_handle.serviceClient<sim_msgs::MoveArm>("/panda1/move_pose_target");
    move_arm_pose_client[AGENT::HUMAN] = node_handle.serviceClient<sim_msgs::MoveArm>("/human_hand/move_hand_pose_target");
    move_arm_named_client[AGENT::ROBOT] = node_handle.serviceClient<sim_msgs::MoveArm>("/panda1/move_named_target");
    move_arm_named_client[AGENT::HUMAN] = node_handle.serviceClient<sim_msgs::MoveArm>("/human_hand/move_hand_named_target");

    attach_plg_client[AGENT::ROBOT] = node_handle.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
    attach_plg_client[AGENT::HUMAN] = node_handle.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
    detach_plg_client[AGENT::ROBOT] = node_handle.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
    detach_plg_client[AGENT::HUMAN] = node_handle.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");

    ros::ServiceServer reset_world_service = node_handle.advertiseService("reset_world", reset_world_server);
    get_world_properties = node_handle.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");

    ros::Publisher step_over_pub = node_handle.advertise<std_msgs::Empty>("/step_over", 10);
    std_msgs::Empty empty_msg;

    get_model_state_client[AGENT::ROBOT] = node_handle.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    get_model_state_client[AGENT::HUMAN] = node_handle.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    set_model_state_client[AGENT::ROBOT] = node_handle.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    set_model_state_client[AGENT::HUMAN] = node_handle.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    ros::ServiceClient gazebo_start_client = node_handle.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");

    ros::AsyncSpinner spinner(4);
    spinner.start();

    std_srvs::Empty empty_srv;
    ros::service::waitForService("/gazebo/unpause_physics");
    gazebo_start_client.call(empty_srv);
    
    ros::service::waitForService("/panda1/move_pose_target");
    ros::service::waitForService("/human_hand/move_hand_pose_target");
    ros::service::waitForService("/panda1/move_named_target");

    home_agents();

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