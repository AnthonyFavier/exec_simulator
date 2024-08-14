#include "sim_controller.h"

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
ros::Publisher head_cmd_pub;
ros::Publisher prompt_pub;
ros::Publisher h_control_camera_pub;

ros::Publisher robot_action_done_pub;
ros::Publisher human_action_done_pub;

ros::Publisher move_human_body_pub;

ros::ServiceClient set_link_state_client;
ros::ServiceClient set_link_properties_client;

ros::ServiceClient set_synchro_step_client;

ros::ServiceClient hide_move_buttons_client;
ros::ServiceClient set_question_buttons_client;

ros::Publisher sound_player_pub;

bool g_step_synchro_on = true;


const double tolerance = 0.01;
const double z_offset_grasp = 0.05;

bool g_h_start_moving = false;
bool g_r_start_moving = false;

sim_msgs::BoxTypes g_box_types;


//  DOMAIN DESCRIPTION  //
std::map<std::string, geometry_msgs::Pose> locations =
    {
        {"box_1_R",     make_pose(make_point(0.73, -0.14, 1.15),        make_quaternion())},
        {"box_2_R",     make_pose(make_point(0.73, 0.17, 1.15),         make_quaternion())},
        {"box_3_R",     make_pose(make_point(0.73, 0.5, 1.15),          make_quaternion())},
        {"box_1_H",     make_pose(make_point(0.97, -0.07, 1.15),        make_quaternion())},
        {"box_2_H",     make_pose(make_point(0.97, 0.24, 1.15),         make_quaternion())},
        {"box_3_H",     make_pose(make_point(0.97, 0.55, 1.15),         make_quaternion())},
};
std::map<std::string, geometry_msgs::Pose> init_poses =
    {
        {"w1",                make_pose(make_point(8.5, -0.4, 0.75),    make_quaternion())},
        {"r1",                make_pose(make_point(0.5, -0.4, 0.75),    make_quaternion())},
        {"y1",                make_pose(make_point(0.5, -0.65, 0.75),   make_quaternion())},
        {"g1",                make_pose(make_point(0.85, -0.45, 0.75),  make_quaternion())},
        {"g2",                make_pose(make_point(0.5, -0.65, 0.85),   make_quaternion())},
};

std::map<std::string, int> nb_dropped_box = 
{
    {"box_1_R", 0},  
    {"box_2_R", 0},  
    {"box_3_R", 0},  
};


enum HUMAN_STATE{AT_MAIN_LOOK_MAIN, AT_MAIN_LOOK_SIDE, AT_SIDE_LOOK_SIDE, AT_SIDE_LOOK_MAIN};
HUMAN_STATE g_human_state = HUMAN_STATE::AT_MAIN_LOOK_MAIN;
geometry_msgs::Pose hand_pose_AT_MAIN_LOOK_MAIN = make_pose(make_point(1.48,  0.5, 0.87),  make_quaternion_RPY(0,0,M_PI));
geometry_msgs::Pose hand_pose_AT_MAIN_LOOK_SIDE = make_pose(make_point(3.72, -0.5, 0.87),  make_quaternion_RPY(0,0,0));
geometry_msgs::Pose hand_pose_AT_SIDE_LOOK_MAIN = make_pose(make_point(5.48 ,  0.5, 0.87),  make_quaternion_RPY(0,0,M_PI));
geometry_msgs::Pose hand_pose_AT_SIDE_LOOK_SIDE = make_pose(make_point(7.72, -0.5, 0.87),  make_quaternion_RPY(0,0,0));


// Deprecated for epistemic
class DropZone
{
public:
    DropZone(geometry_msgs::Point point)
    {
        m_pose = make_pose( point, make_quaternion() );
        m_occupied = false;
        m_obj_name = "";
    }
    void setOccupied(bool v)
    {
        m_occupied = v;
    }
    bool isOccupied()
    {
        return m_occupied;
    }
    geometry_msgs::Pose getPose()
    {
        return m_pose;
    }

private:
    bool m_occupied;
    std::string m_obj_name;
    geometry_msgs::Pose m_pose;
};
std::vector<DropZone> g_center_drop_zones;
void init_drop_zones()
{
    g_center_drop_zones.push_back( DropZone(make_point(0.86, -0.20, 0.75)) );
    g_center_drop_zones.push_back( DropZone(make_point(0.86, -0.35, 0.75)) );
    g_center_drop_zones.push_back( DropZone(make_point(0.86, -0.50, 0.75)) );
}


// ************************* HIGH LEVEL ACTIONS **************************** //
void PickName(AGENT agent, std::string obj_name)
{
    ROS_INFO("\t%s PICK START %s", get_agent_str(agent).c_str(), obj_name.c_str());

    g_holding[agent] = obj_name;

    geometry_msgs::Pose obj_pose;
    if(init_poses.find(obj_name)!=init_poses.end())
    {
        set_obj_position(agent, obj_name, init_poses[obj_name]);
        obj_pose = init_poses[obj_name];
    }
    else
    {
        /* GET OBJ POSE */
        gazebo_msgs::GetModelState srv;
        srv.request.model_name = obj_name;
        if (!get_model_state_client[agent].call(srv) || !srv.response.success)
            throw ros::Exception("Calling service get_model_state failed...");
        obj_pose = srv.response.pose;
    }
    show_pose(obj_pose);
    set_obj_pose(agent, obj_name, obj_pose);

    /* MOVE ROBOT HEAD */
    robot_head_follow_obj(agent, obj_name);

    geometry_msgs::Pose mid_pose = make_pose(make_point(1.4, -0.4, 0.84), make_quaternion());

    /* MOVE ARM TO OBJ */
    if(agent==AGENT::HUMAN and obj_name=="g1")
        move_pose_target(agent, mid_pose);
    move_pose_target(agent, obj_pose);

    /* GRAB OBJ */
    set_mass_obj(obj_name, false);
    grab_obj(agent, obj_name);

    /* HOME POSITION */
    if(agent==AGENT::HUMAN and obj_name=="g1")
        move_pose_target(agent, mid_pose);
    move_home(agent);

    /* MOVE ROBOT HEAD */
    robot_head_follow_human(agent);

    ROS_INFO("\t%s PICK END", get_agent_str(agent).c_str());
}

void PickNameConcu(AGENT agent, std::string obj_name)
{
    ROS_INFO("\t%s PICK START %s", get_agent_str(agent).c_str(), obj_name.c_str());

    g_holding[agent] = obj_name;

    geometry_msgs::Pose obj_pose;
    if(init_poses.find(obj_name)!=init_poses.end())
    {
        set_obj_position(agent, obj_name, init_poses[obj_name]);
        obj_pose = init_poses[obj_name];
    }
    else
    {
        /* GET OBJ POSE */
        gazebo_msgs::GetModelState srv;
        srv.request.model_name = obj_name;
        if (!get_model_state_client[agent].call(srv) || !srv.response.success)
            throw ros::Exception("Calling service get_model_state failed...");
        obj_pose = srv.response.pose;
    }
    show_pose(obj_pose);

    /* MOVE ROBOT HEAD */
    robot_head_follow_obj(agent, obj_name);

    geometry_msgs::Pose mid_pose = make_pose(make_point(1.4, -0.4, 0.84), make_quaternion());

    /* MOVE ARM TO OBJ */
    if(agent==AGENT::HUMAN and obj_name=="g1")
        move_pose_target(agent, mid_pose);
    move_pose_target(agent, obj_pose);

    /* GRAB OBJ */
    set_mass_obj(obj_name, false);
    set_obj_pose(agent, obj_name, obj_pose);
    grab_obj(agent, obj_name);

    ROS_INFO("\t%s PICK_CONCU END", get_agent_str(agent).c_str());
}

void PlacePose(AGENT agent, geometry_msgs::Pose pose)
{
    ROS_INFO("\t%s PLACE_POSE START", get_agent_str(agent).c_str());

    /* MOVE ROBOT HEAD */
    robot_head_follow_pose(agent, pose.position);

    /* MOVE ARM TO POSE */
    move_pose_target(agent, pose);

    /* DROP OBJ */
    std::string obj_name = g_holding[agent];
    drop(agent, obj_name);
    set_obj_pose(agent, obj_name, pose);
    set_mass_obj(obj_name, true);

    /* HOME POSITION */
    move_home(agent);

    /* MOVE ROBOT HEAD */
    robot_head_follow_human(agent);

    ROS_INFO("\t%s PLACE_POSE END", get_agent_str(agent).c_str());
}

void PlaceLocation(AGENT agent, std::string location)
{
    geometry_msgs::Pose pose = locations[location];

    if(agent==AGENT::ROBOT)
    {
        if(nb_dropped_box[location]!=0)
            pose.position.y += 0.14;
        nb_dropped_box[location]++;
    }

    PlacePose(agent, pose);
}

void PlaceLocationConcu(AGENT agent, std::string location)
{
    geometry_msgs::Pose pose = locations[location];

    if(agent==AGENT::ROBOT)
    {
        if(nb_dropped_box[location]!=0)
            pose.position.y += 0.14;
        nb_dropped_box[location]++;
    }

    ROS_INFO("\t%s PLACE_POSE START", get_agent_str(agent).c_str());

    /* MOVE ROBOT HEAD */
    robot_head_follow_pose(agent, pose.position);

    /* MOVE ARM TO POSE */
    move_pose_target(agent, pose);

    /* DROP OBJ */
    std::string obj_name = g_holding[agent];
    drop(agent, obj_name);
    set_obj_pose(agent, obj_name, pose);
    set_mass_obj(obj_name, true);

    ROS_INFO("\t%s PLACE_POSE END", get_agent_str(agent).c_str());
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



void TurnAround()
{
    // compute new hand pose
    geometry_msgs::Pose new_hand_pose ;
    switch(g_human_state)
    {
        case HUMAN_STATE::AT_MAIN_LOOK_MAIN:{
            g_human_state = AT_MAIN_LOOK_SIDE;
            new_hand_pose = hand_pose_AT_MAIN_LOOK_SIDE;
            break;}

        case HUMAN_STATE::AT_MAIN_LOOK_SIDE:{
            g_human_state = AT_MAIN_LOOK_MAIN;
            new_hand_pose = hand_pose_AT_MAIN_LOOK_MAIN;
            break;}

        case HUMAN_STATE::AT_SIDE_LOOK_SIDE:{
            g_human_state = AT_SIDE_LOOK_MAIN;
            new_hand_pose = hand_pose_AT_SIDE_LOOK_MAIN;
            break;}

        case HUMAN_STATE::AT_SIDE_LOOK_MAIN:{
            g_human_state = AT_SIDE_LOOK_SIDE;
            new_hand_pose = hand_pose_AT_SIDE_LOOK_SIDE;
            break;}
    }    


    // Turn human body
    std_msgs::Int32 msg_body;
    msg_body.data = 0;
    move_human_body_pub.publish(msg_body);

    // turn around camera
    std_msgs::Int32 msg;
    msg.data = 0;
    h_control_camera_pub.publish(msg);

    // Move hand
    move_pose_target(AGENT::HUMAN, new_hand_pose, false, 3);

}

void MoveForward()
{
    // compute new hand pose
    geometry_msgs::Pose new_hand_pose ;
    switch(g_human_state)
    {
        case HUMAN_STATE::AT_MAIN_LOOK_SIDE:{
            g_human_state = AT_SIDE_LOOK_SIDE;
            new_hand_pose = hand_pose_AT_SIDE_LOOK_SIDE;
            break;}

        case HUMAN_STATE::AT_SIDE_LOOK_MAIN:{
            g_human_state = AT_MAIN_LOOK_MAIN;
            new_hand_pose = hand_pose_AT_MAIN_LOOK_MAIN;
            break;}
    }


    // Move human body
    std_msgs::Int32 msg_body;
    msg_body.data = 1;
    move_human_body_pub.publish(msg_body);

    // move forward camera
    std_msgs::Int32 msg;
    msg.data = 1;
    h_control_camera_pub.publish(msg);

    // Move hand
    move_pose_target(AGENT::HUMAN, new_hand_pose, false, 2);

}

void Ask(std::string obj_name, std::string location, sim_msgs::CanPlaceAnswers answers)
{
    // Look at human
    robot_head_follow_human(AGENT::ROBOT);

    // Retrieve answer
    bool answer;
    if(location=="box_1")
        answer = answers.box_1;
    else if(location=="box_2")
        answer = answers.box_2;
    else if(location=="box_3")
        answer = answers.box_3;
    else
        ROS_ERROR("WEIRD...");

    // Prompt answer 
    std_msgs::String prompt_msg;
    if(answer)
        prompt_msg.data = "  Yes, " + location + " is empty.";
    else
        prompt_msg.data = "  No, " + location + " is full...";

    ROS_WARN("%s", prompt_msg.data.c_str());

    // Speak
    std_msgs::String msg;
    std::string box_id(1, location.back());
    std::string answer_str = answer ? "yes" : "no";
    msg.data = answer_str+"_box"+box_id;
    sound_player_pub.publish(msg);

    // Delay
    ROS_WARN("Waiting N seconds (N*2s sim time)...");
    float N = 2;
    ros::Rate loop(2);
    ros::Time start = ros::Time::now();
    while(ros::Time::now() - start < ros::Duration(N*2))
    {
        prompt_pub.publish(prompt_msg);
        loop.sleep();
    }

    ROS_WARN("Done !");
}

void Com(std::string location, bool empty)
{
    robot_head_follow_human(AGENT::ROBOT);

    // Prompt answer 
    std_msgs::String prompt_msg;

    std::string box_id(1, location.back());
    std::string empty_str = empty ? "empty" : "full";

    prompt_msg.data = "  Box " + box_id + " is " + empty_str + ".";

    // Speak
    std_msgs::String msg;
    msg.data = "box"+box_id+"_"+empty_str;
    sound_player_pub.publish(msg);

    ROS_WARN("%s", prompt_msg.data.c_str());

    // Delay
    ROS_WARN("Waiting N seconds (N*2s sim time)...");
    float N = 2;
    ros::Rate loop(2);
    ros::Time start = ros::Time::now();
    while(ros::Time::now() - start < ros::Duration(N*2))
    {
        prompt_pub.publish(prompt_msg);
        loop.sleep();
    }

    ROS_WARN("Done !");
}


//  MANAGE ACTIONS + EVENTS + SIGNALS  //
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
    case sim_msgs::Action::PICK_OBJ_NAME:
        name += "Pick(" + action.obj_name + ")";
        break;
    case sim_msgs::Action::PLACE_OBJ_NAME:
        name += "Place(" + action.location + "," + action.obj_name + ")";
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

std::string get_str_agent(AGENT agent)
{
    if(agent==AGENT::ROBOT)
        return "R";
    else if(agent==AGENT::HUMAN)
        return "H";
    else
        return "";
}

std::string get_str_action(int type)
{
    switch(type)
    {
        case 0:
            return "PASSIVE";
            break;
        case 1:
            return "PICK_OBJ";
            break;
        case 2:
            return "PLACE_OBJ";
            break;
        case 3:
            return "PUSH";
            break;
        case 4:
            return "OPEN_BOX";
            break;
        case 5:
            return "DROP";
            break;
        case 6:
            return "PICK_OBJ_NAME";
            break;
        case 7:
            return "PLACE_OBJ_NAME";
            break;
        case 8:
            return "TURN_AROUND";
            break;
        case 9:
            return "MOVE_FORWARD";
            break;
        case 10:
            return "ASK";
            break;

        default:
            return "";
            break;
    }
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
        // ROS_INFO("Manage %s action %s", get_str_agent(agent), get_str_action(action.type));
        ROS_INFO_STREAM("Manage " << get_str_agent(agent) << " action " << get_str_action(action.type) << ".");

        if (sim_msgs::Action::PASSIVE == action.type)
            BePassive(agent);
        else if(sim_msgs::Action::TURN_AROUND == action.type)
        {
            action_received[agent] = true;
            // send visual signal
            sim_msgs::Signal sgl;
            sgl.id = action.id;
            sgl.type = sim_msgs::Signal::S_HA;
            visual_signals_pub[agent].publish(sgl);
            ROS_INFO("Start signal SENT");
            
            TurnAround();

            send_visual_signal_action_over(agent, action);
            action_done[agent] = true;
        }
        else if(sim_msgs::Action::MOVE_FORWARD == action.type)
        {
            action_received[agent] = true;
            // send visual signal
            sim_msgs::Signal sgl;
            sgl.id = action.id;
            sgl.type = sim_msgs::Signal::S_HA;
            visual_signals_pub[agent].publish(sgl);
            ROS_INFO("Start signal SENT");
            
            MoveForward();

            send_visual_signal_action_over(agent, action);
            action_done[agent] = true;
        }
        else if(sim_msgs::Action::ASK == action.type)
        {
            action_received[agent] = true;
            sim_msgs::Signal sgl;
            sgl.id = action.id;
            sgl.type = sim_msgs::Signal::S_HA;
            visual_signals_pub[agent].publish(sgl);
            ROS_INFO("Start signal SENT for ASKING");

            ros::Duration(0.2).sleep();

            Ask(action.obj_name, action.location, action.answers);

            send_visual_signal_action_over(agent, action);
            action_done[agent] = true;
        }
        else if(sim_msgs::Action::COM == action.type)
        {
            action_received[agent] = true;
            sim_msgs::Signal sgl;
            sgl.id = action.id;
            sgl.type = sim_msgs::Signal::S_RA;
            visual_signals_pub[agent].publish(sgl);
            ROS_INFO("Start signal SENT for ASKING");

            ros::Duration(0.2).sleep();

            Com(action.location, action.empty);

            send_visual_signal_action_over(agent, action);
            action_done[agent] = true;
        }
        else
        {
            // log event
            sim_msgs::EventLog event;
            event.timestamp = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
            event.name = compute_event_name(agent, action, true);
            event_log_pub[agent].publish(event);
            ROS_INFO("Start EVENT SENT");

            action_received[agent] = true;
            std::thread t1(send_visual_signal_action_start, agent, action);
            switch (action.type)
            {
            case sim_msgs::Action::PICK_OBJ_NAME:
                if (action.obj_name == "")
                    ROS_ERROR("%d Missing obj_name in action msg!", agent);
                else
                    PickName(agent, action.obj_name);
                break;
            case sim_msgs::Action::PICK_OBJ_NAME_CONCU:
                if (action.obj_name == "")
                    ROS_ERROR("%d Missing obj_name in action msg!", agent);
                else
                    PickNameConcu(agent, action.obj_name);
                break;
            case sim_msgs::Action::PLACE_OBJ_NAME:
                if (action.location == "")
                    ROS_ERROR("%d Missing location in action msg!", agent);
                else
                    PlaceLocation(agent, action.location);
                break;
            case sim_msgs::Action::PLACE_OBJ_NAME_CONCU:
                if (action.location == "")
                    ROS_ERROR("%d Missing location in action msg!", agent);
                else
                    PlaceLocationConcu(agent, action.location);
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
// *********************************************************** //



// ************************* LOW LEVEL ACTIONS **************************** //
void move_pose_target(AGENT agent, const geometry_msgs::Pose &pose_target, bool human_home /* = false */, int hand_speed /* = 1 */)
{
    ROS_INFO_STREAM("\t\t" << get_agent_str(agent).c_str() << std::setprecision(4) << " MOVE_POSE_TARGET START (" 
        << pose_target.position.x << ", " << pose_target.position.y << ", " << pose_target.position.z << ")" <<
        " (" << pose_target.orientation.x << ", " << pose_target.orientation.y << ", " << pose_target.orientation.z << ", " << pose_target.orientation.w << ")");
    sim_msgs::MoveArm srv;
    srv.request.pose_target = pose_target;

    srv.request.hand_speed = hand_speed;

    /* OFFSET POSE if human and not home */
    if (!isRobot(agent) && !human_home)
        srv.request.pose_target.position.z += z_offset_grasp;

    if (!move_arm_pose_client[agent].call(srv) || !srv.response.success)
        throw ros::Exception("Calling service move_arm_pose_target failed...");
    ROS_INFO_STREAM("\t\t" << get_agent_str(agent).c_str() << std::setprecision(4) << " MOVE_POSE_TARGET END (" 
        << pose_target.position.x << ", " << pose_target.position.y << ", " << pose_target.position.z << ")" <<
        " (" << pose_target.orientation.x << ", " << pose_target.orientation.y << ", " << pose_target.orientation.z << ", " << pose_target.orientation.w << ")");
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
    {
        switch(g_human_state)
        {
            case HUMAN_STATE::AT_MAIN_LOOK_MAIN:
                move_pose_target(agent, hand_pose_AT_MAIN_LOOK_MAIN, true);
                break;
            case HUMAN_STATE::AT_SIDE_LOOK_SIDE:
                move_pose_target(agent, hand_pose_AT_SIDE_LOOK_SIDE, true);
                break;
            default:
                break;
        }
    }
}

void move_named_target(AGENT agent, const std::string &named_target)
{
    ROS_INFO_STREAM("\t\t" << get_agent_str(agent).c_str() << " MOVE_NAMED_TARGET START (" << named_target << ")");
    sim_msgs::MoveArm srv;
    srv.request.named_target = named_target;
    if (!move_arm_named_client[agent].call(srv) || !srv.response.success)
        throw ros::Exception("Calling service move_arm_named_target failed...");
    ROS_INFO_STREAM("\t\t" << get_agent_str(agent).c_str() << " MOVE_NAMED_TARGET END (" << named_target << ")");
}

void grab_obj(AGENT agent, const std::string &object)
{
    /*
    Attach object to agent end effector
    */

    ROS_INFO("\t\t%s GRAB_OBJ START", get_agent_str(agent).c_str());

    sleep(0.5);

    gazebo_ros_link_attacher::Attach srv;
    if (agent == AGENT::ROBOT)
    {
        srv.request.model_name_1 = ROBOT_ATTACH_MODEL_NAME;
        srv.request.link_name_1 = ROBOT_ATTACH_LINK_NAME;
    }
    else if (agent == AGENT::HUMAN)
    {
        sleep(0.5);
        srv.request.model_name_1 = HUMAN_ATTACH_MODEL_NAME;
        srv.request.link_name_1 = HUMAN_ATTACH_LINK_NAME;
    }
    srv.request.model_name_2 = object;
    srv.request.link_name_2 = object+"_link";
    if (!attach_plg_client[agent].call(srv) || !srv.response.ok)
        throw ros::Exception("Calling service attach_plg_client failed...");

    ROS_INFO("\t\t%s GRAB_OBJ END", get_agent_str(agent).c_str());
}

void drop(AGENT agent, const std::string &object)
{
    /* 
    Detach object from agent's end effector
    */

    ROS_INFO("\t\t%s DROP START", get_agent_str(agent).c_str());

    sleep(0.5);

    gazebo_ros_link_attacher::Attach srv;
    if (agent == AGENT::ROBOT)
    {
        srv.request.model_name_1 = ROBOT_ATTACH_MODEL_NAME;
        srv.request.link_name_1 = ROBOT_ATTACH_LINK_NAME;
    }
    else if (agent == AGENT::HUMAN)
    {
        sleep(0.5);
        srv.request.model_name_1 = HUMAN_ATTACH_MODEL_NAME;
        srv.request.link_name_1 = HUMAN_ATTACH_LINK_NAME;
    }
    srv.request.model_name_2 = object;
    srv.request.link_name_2 = object+"_link";
    std::cout << agent << " " << srv.request.model_name_1 << " " << srv.request.link_name_1 << " " << srv.request.model_name_2 << " " << srv.request.link_name_2 << std::endl;
    if (!detach_plg_client[agent].call(srv) || !srv.response.ok)
        throw ros::Exception("Calling service detach_plg_client failed...");

    g_holding[agent] = "";

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

void set_obj_position(AGENT agent, std::string obj_name, geometry_msgs::Pose pose)
{
    gazebo_msgs::GetModelState srv_get;
    srv_get.request.model_name = obj_name;
    get_model_state_client[agent].call(srv_get);

    gazebo_msgs::SetModelState srv;
    srv.request.model_state.model_name = obj_name;
    srv.request.model_state.pose.position = pose.position;
    srv.request.model_state.pose.orientation = srv_get.response.pose.orientation;
    set_model_state_client[agent].call(srv);
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

void set_mass_obj(std::string obj_name, bool enable)
{
    gazebo_msgs::SetLinkProperties srv;
    srv.request.link_name = obj_name+"_link";

    if(enable)
    {
        srv.request.gravity_mode = true;
        srv.request.mass = 0.1;
        srv.request.ixx = 0.001;
        srv.request.iyy = 0.001;
        srv.request.izz = 0.001;
    }
    else
    {
        srv.request.gravity_mode = false;
        srv.request.mass = 0.000001;
        srv.request.ixx = 0.000001;
        srv.request.iyy = 0.000001;
        srv.request.izz = 0.000001;
    }

    set_link_properties_client.call(srv);
}

// ****************************** UTILS ******************************* //

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


// ****************************** CALLBACKS ******************************* //

void robot_action_cb(const sim_msgs::Action &msg)
{
    manage_action(AGENT::ROBOT, msg);
}

void human_action_cb(const sim_msgs::Action &msg)
{
    manage_action(AGENT::HUMAN, msg);
}

void r_home_cb(std_msgs::Empty msg)
{
    move_home(AGENT::ROBOT);
}

void h_home_cb(std_msgs::Empty msg)
{
    move_home(AGENT::HUMAN);
}

void h_start_moving_cb(std_msgs::Empty msg)
{
    g_h_start_moving = true;
}

void r_start_moving_cb(std_msgs::Empty msg)
{
    g_r_start_moving = true;
}

// ************************************************************************ //

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
    ROS_INFO("Start reset...");


    ROS_INFO("\tReset drop zones");
    for (auto it = g_center_drop_zones.begin(); it != g_center_drop_zones.end(); it++)
        (*it).setOccupied(false);

    // Get world models
    // ROS_INFO("\tGet world models");
    // gazebo_msgs::GetWorldProperties srv;
    // if (!get_world_properties.call(srv) || !srv.response.success)
    //     throw ros::Exception("Calling get_world_properties failed...");

    // Detach 
    ROS_INFO("\tDetach cubes");
    gazebo_ros_link_attacher::Attach srv_attach;
    gazebo_msgs::SetModelState srv_set;

    for(auto const& x: init_poses)
    {
        std::string obj_name = x.first;
        geometry_msgs::Pose obj_pose = x.second;

        // Detach obj from robot
        srv_attach.request.model_name_2 = obj_name;
        srv_attach.request.link_name_2 = obj_name+"_link";
        srv_attach.request.model_name_1 = ROBOT_ATTACH_MODEL_NAME;
        srv_attach.request.link_name_1 = ROBOT_ATTACH_LINK_NAME;
        detach_plg_client[AGENT::ROBOT].call(srv_attach);

        // Detach obj from human hand
        srv_attach.request.model_name_1 = HUMAN_ATTACH_MODEL_NAME;
        srv_attach.request.link_name_1 = HUMAN_ATTACH_LINK_NAME;
        detach_plg_client[AGENT::HUMAN].call(srv_attach);
    }

    // Reset robot head
    ROS_INFO("\tReset robot head");
    sim_msgs::HeadCmd head_cmd;
    head_cmd.type = sim_msgs::HeadCmd::RESET;
    head_cmd_pub.publish(head_cmd);

    // Home agents
    ROS_INFO("\tHome agent");
    home_agents();
    action_received[AGENT::ROBOT] = false;
    action_received[AGENT::HUMAN] = false;
    action_done[AGENT::ROBOT] = false;
    action_done[AGENT::HUMAN] = false;

    // Reset prompt
    ROS_INFO("\tReset prompt");
    std_msgs::String prompt_msg;
    prompt_pub.publish(prompt_msg);

    // Reset move buttons
    ROS_INFO("\tReset move buttons");
    std_srvs::Empty empty_srv;
    hide_move_buttons_client.call(empty_srv);

    // Reset Question buttons
    ROS_INFO("\tReset question buttons");
    sim_msgs::SetQuestionButtons srv_question;
    srv_question.request.q1 = false; 
    srv_question.request.q2 = false; 
    srv_question.request.q3 = false; 
    srv_question.request.q4 = false; 
    srv_question.request.q5 = false; 
    set_question_buttons_client.call(srv_question);

    // Reset Camera
    ROS_INFO("\tReset camera");
    std_msgs::Int32 camera_reset_msg;
    camera_reset_msg.data = -1;
    h_control_camera_pub.publish(camera_reset_msg);

    // Set synchro step
    g_step_synchro_on = true;

    // Reset human body
    std_msgs::Int32 msg_body;
    msg_body.data = -1;
    move_human_body_pub.publish(msg_body);

    // Reset nb_dropped
    nb_dropped_box["box_1_R"] = 0;  
    nb_dropped_box["box_2_R"] = 0;  
    nb_dropped_box["box_3_R"] = 0; 

    // Set 3 cubes
    set_3_cubes_server(empty_srv.request, empty_srv.response);

    ROS_INFO("World reset ok");

    return true;
}

bool go_init_pose_server(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    move_named_target(AGENT::ROBOT, "init");
    return true;
}

void go_init_pose_cb(const std_msgs::Empty &msg)
{
    move_named_target(AGENT::ROBOT, "init");
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

bool set_synchro_step_server(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res)
{
    g_step_synchro_on = req.data;

    return true;
}

// Box type (OPAQUE / TRANSPARENT)
bool set_box_types_server(sim_msgs::SetBoxTypesRequest &req, sim_msgs::SetBoxTypesResponse &res)
{
    gazebo_msgs::SetModelState srv_set;
    geometry_msgs::Pose far_pose = make_pose(make_point(0, 0, -1), make_quaternion_RPY(0,0,0));

    geometry_msgs::Pose box_1_cover_pose = make_pose(make_point(0.85, -0.07, 0.7),   make_quaternion());
    geometry_msgs::Pose box_2_cover_pose = make_pose(make_point(0.85, 0.24, 0.7),   make_quaternion());
    geometry_msgs::Pose box_3_cover_pose = make_pose(make_point(0.85, 0.55, 0.7),   make_quaternion());


    g_box_types = req.types;

    /* Box 1 */
    srv_set.request.model_state.model_name = "box_1_cover";
    srv_set.request.model_state.pose = (req.types.box_1==sim_msgs::BoxTypes::OPAQUE) ? box_1_cover_pose : far_pose;
    set_model_state_client[AGENT::ROBOT].call(srv_set);

    /* Box 2 */
    srv_set.request.model_state.model_name = "box_2_cover";
    srv_set.request.model_state.pose = (req.types.box_2==sim_msgs::BoxTypes::OPAQUE) ? box_2_cover_pose : far_pose;
    set_model_state_client[AGENT::ROBOT].call(srv_set);

    /* Box 3 */
    srv_set.request.model_state.model_name = "box_3_cover";
    srv_set.request.model_state.pose = (req.types.box_3==sim_msgs::BoxTypes::OPAQUE) ? box_3_cover_pose : far_pose;
    set_model_state_client[AGENT::ROBOT].call(srv_set);

    return true;
}
bool get_box_types_server(sim_msgs::GetBoxTypesRequest &req, sim_msgs::GetBoxTypesResponse &res)
{
    res.types = g_box_types;

    return true;
}

bool set_3_cubes_server(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    gazebo_msgs::SetModelState srv_set;

    geometry_msgs::Pose far_pose = make_pose(make_point(0.0, 6.0, 1.0), make_quaternion());

    srv_set.request.model_state.model_name = "r1";
    srv_set.request.model_state.pose = init_poses["r1"];
    set_model_state_client[AGENT::ROBOT].call(srv_set);

    srv_set.request.model_state.model_name = "y1";
    srv_set.request.model_state.pose = init_poses["y1"];
    set_model_state_client[AGENT::ROBOT].call(srv_set);

    srv_set.request.model_state.model_name = "g1";
    srv_set.request.model_state.pose = far_pose;
    set_model_state_client[AGENT::ROBOT].call(srv_set);

    srv_set.request.model_state.model_name = "g2";
    srv_set.request.model_state.pose = far_pose;
    set_model_state_client[AGENT::ROBOT].call(srv_set);

    srv_set.request.model_state.model_name = "w1";
    srv_set.request.model_state.pose = init_poses["w1"];
    set_model_state_client[AGENT::ROBOT].call(srv_set);

    return true;
}

bool set_4_cubes_server(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    gazebo_msgs::SetModelState srv_set;

    geometry_msgs::Pose far_pose = make_pose(make_point(0.0, 6.0, 1.0), make_quaternion());

    srv_set.request.model_state.model_name = "r1";
    srv_set.request.model_state.pose = far_pose;
    set_model_state_client[AGENT::ROBOT].call(srv_set);

    srv_set.request.model_state.model_name = "y1";
    srv_set.request.model_state.pose = init_poses["y1"];
    set_model_state_client[AGENT::ROBOT].call(srv_set);

    srv_set.request.model_state.model_name = "g1";
    srv_set.request.model_state.pose = init_poses["g1"];
    set_model_state_client[AGENT::ROBOT].call(srv_set);

    srv_set.request.model_state.model_name = "g2";
    srv_set.request.model_state.pose = init_poses["g2"];
    set_model_state_client[AGENT::ROBOT].call(srv_set);

    srv_set.request.model_state.model_name = "w1";
    srv_set.request.model_state.pose = init_poses["w1"];
    set_model_state_client[AGENT::ROBOT].call(srv_set);

    return true;
}

int main(int argc, char **argv)
{
    init_drop_zones();

    ros::init(argc, argv, "sim_controller");
    ros::NodeHandle node_handle;

    ros::Subscriber robot_action = node_handle.subscribe("/robot_action", 1, robot_action_cb);
    ros::Subscriber human_action = node_handle.subscribe("/human_action", 1, human_action_cb);

    ros::Subscriber r_home = node_handle.subscribe("/r_home", 1, r_home_cb);
    r_home_pub = node_handle.advertise<std_msgs::Empty>("/r_home", 1);

    ros::Subscriber h_home = node_handle.subscribe("/h_home", 1, h_home_cb);
    h_home_pub = node_handle.advertise<std_msgs::Empty>("/h_home", 1);

    h_control_camera_pub = node_handle.advertise<std_msgs::Int32>("/control_camera", 1);

    const std::string r_move_pose_srv_name("/move_pose_target");
    const std::string r_move_named_topic_name("/move_named_target");
    const std::string r_head_cmd_topic_name("/tiago_head_cmd");
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
    // ros::Subscriber go_init_pose_sub = node_handle.subscribe("/go_init_pose", 1, go_init_pose_cb);
    ros::ServiceServer go_idle_pose_service = node_handle.advertiseService("go_idle_pose", go_idle_pose_server);
    ros::ServiceServer go_home_pose_service = node_handle.advertiseService("go_home_pose", go_home_pose_server);
    ros::ServiceServer go_init_pose_service = node_handle.advertiseService("go_init_pose", go_init_pose_server);

    event_log_pub[AGENT::ROBOT] = node_handle.advertise<sim_msgs::EventLog>("/event_log", 10);
    event_log_pub[AGENT::HUMAN] = node_handle.advertise<sim_msgs::EventLog>("/event_log", 10);
    visual_signals_pub[AGENT::ROBOT] = node_handle.advertise<sim_msgs::Signal>("/robot_visual_signals", 10);
    visual_signals_pub[AGENT::HUMAN] = node_handle.advertise<sim_msgs::Signal>("/human_visual_signals", 10);

	set_link_state_client = node_handle.serviceClient<gazebo_msgs::SetLinkState>("/gazebo/set_link_state");
	
    set_link_properties_client = node_handle.serviceClient<gazebo_msgs::SetLinkProperties>("/gazebo/set_link_properties");

    ros::Publisher step_over_pub = node_handle.advertise<std_msgs::Empty>("/step_over", 10);
    std_msgs::Empty empty_msg;

    get_model_state_client[AGENT::ROBOT] = node_handle.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    get_model_state_client[AGENT::HUMAN] = node_handle.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    set_model_state_client[AGENT::ROBOT] = node_handle.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    set_model_state_client[AGENT::HUMAN] = node_handle.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    head_cmd_pub = node_handle.advertise<sim_msgs::HeadCmd>(r_head_cmd_topic_name, 10);

    prompt_pub = node_handle.advertise<std_msgs::String>("/simu_prompt", 10);

    robot_action_done_pub = node_handle.advertise<std_msgs::Empty>("/robot_action_done", 1);
    human_action_done_pub = node_handle.advertise<std_msgs::Empty>("/human_action_done", 1);

    move_human_body_pub = node_handle.advertise<std_msgs::Int32>("/move_human_body", 1);
    
    hide_move_buttons_client = node_handle.serviceClient<std_srvs::Empty>("/hide_move_buttons");
    set_question_buttons_client = node_handle.serviceClient<sim_msgs::SetQuestionButtons>("/set_question_buttons");

    ros::ServiceClient gazebo_start_client = node_handle.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");

    // set_synchro_step_client = node_handle.serviceClient<std_srvs::SetBool>("/set_synchro_step");
    ros::ServiceServer set_synchro_step_service = node_handle.advertiseService("set_synchro_step", set_synchro_step_server);
    
    ros::ServiceServer set_box_types_service = node_handle.advertiseService("set_box_types", set_box_types_server);
    ros::ServiceServer get_box_types_service = node_handle.advertiseService("get_box_types", get_box_types_server);
    
    ros::ServiceServer set_3_cubes_service = node_handle.advertiseService("/set_3_cubes", set_3_cubes_server); 
    ros::ServiceServer set_4_cubes_service = node_handle.advertiseService("/set_4_cubes", set_4_cubes_server); 

    ros::Subscriber r_start_moving_sub = node_handle.subscribe("/r_start_moving", 1, r_start_moving_cb);
    ros::Subscriber h_start_moving_sub = node_handle.subscribe("/h_start_moving", 1, h_start_moving_cb);

    sound_player_pub = node_handle.advertise<std_msgs::String>("/sound_player", 1);

    ros::AsyncSpinner spinner(10);
    spinner.start();

    std_srvs::Empty empty_srv;
    ros::service::waitForService("/gazebo/unpause_physics");
    gazebo_start_client.call(empty_srv);

    ros::service::waitForService(r_move_pose_srv_name);
    ros::service::waitForService(h_move_pose_srv_name);
    ros::service::waitForService(r_head_cmd_ready_name);

    // home_agents();

    ROS_INFO("Controllers Ready!");


    ros::Rate loop(50);
    while (ros::ok())
    {

        if(g_step_synchro_on)
        {
            // ROS_INFO("%d-%d %d-%d", action_received[AGENT::ROBOT], action_done[AGENT::ROBOT], action_received[AGENT::HUMAN], action_done[AGENT::HUMAN]);
            if((action_received[AGENT::ROBOT] && action_done[AGENT::ROBOT] && action_received[AGENT::HUMAN] && action_done[AGENT::HUMAN]) 
            || (action_received[AGENT::ROBOT] && action_done[AGENT::ROBOT] && !action_received[AGENT::HUMAN] && !action_done[AGENT::HUMAN])
            || (!action_received[AGENT::ROBOT] && !action_done[AGENT::ROBOT] && action_received[AGENT::HUMAN] && action_done[AGENT::HUMAN]))
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

        }

        else
        {
            if(action_received[AGENT::ROBOT] && action_done[AGENT::ROBOT])
            {
                robot_action_done_pub.publish(empty_msg);
                action_received[AGENT::ROBOT] = false;
                action_done[AGENT::ROBOT] = false;
            }

            if(action_received[AGENT::HUMAN] && action_done[AGENT::HUMAN])
            {
                human_action_done_pub.publish(empty_msg);
                action_received[AGENT::HUMAN] = false;
                action_done[AGENT::HUMAN] = false;
            }
        }

        loop.sleep();
    }

    return 0;
}