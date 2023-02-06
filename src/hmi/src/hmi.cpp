#include <ros/ros.h>
#include <termios.h>
#include <vector>
#include "sim_msgs/VHA.h"
#include "std_msgs/Int32.h"
#include "std_srvs/Empty.h"

int getch()
{
    static struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt); // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); // apply new settings

    int c = getchar(); // read character (non-blocking)

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // restore old settings
    return c;
}

std::vector<std::string> valid_actions;

void clear_line()
{
    std::cout << "\33[2K\r";
}

void print_actions()
{
    clear_line();
    std::string str ="";
    for(int i=0; i<valid_actions.size(); i++)
    {
        str += std::to_string(i) + " (" + valid_actions[i] + ")";
        if(i<valid_actions.size()-1)
            str += " - ";
    }
    std::cout << str;
}

void print_actions_bis()
{
    std::string str ="";
    for(int i=0; i<valid_actions.size(); i++)
    {
        str += std::to_string(i) + " " + valid_actions[i];
        if(i<valid_actions.size()-1)
            str += " - ";
    }
    std::cout << str << "\nSelect any displayed action:" << std::endl;
}


bool last_type_received_is_first=false;
bool waiting_new_step=false;
bool first = false;
void vha_cb(const sim_msgs::VHA msg)
{
    first = true;
    valid_actions.clear();
    for(int i=0; i<msg.valid_human_actions.size(); i++)
        valid_actions.push_back(msg.valid_human_actions[i]);
    waiting_new_step = false;
    if(msg.type==msg.START)
        std::cout << "\n" << "Step start human actions:" << "\n";
    else if(msg.type==msg.CONCURRENT)
        std::cout << "\n" << "Concurrent human actions:" << "\n";
    print_actions_bis();
}

bool started(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hmi");
    ros::NodeHandle node_handle;

    ros::Subscriber vha_sub = node_handle.subscribe("/hmi_vha", 5, vha_cb);
    ros::Publisher human_choice_pub = node_handle.advertise<std_msgs::Int32>("/human_choice", 1);

    ros::ServiceServer startup_service = node_handle.advertiseService("hmi_started", started);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::Rate loop(50);
    std::cout << "waiting first vha..." << std::endl;
    while (ros::ok() && !first)
        loop.sleep();

    while (ros::ok())
    {
        int c = getch(); // call your non-blocking input function
        if(!waiting_new_step && c>=48 && c<=57) // is a number
        {
            if(c-48<valid_actions.size())
            {
                std_msgs::Int32 msg;
                msg.data = c - 48;
                std::cout << "Action " << char(c) << " selected !" << std::endl;
                if(valid_actions[c-48]!="LRD()" || !last_type_received_is_first and valid_actions[c-48]=="LRD()")
                    std::cout << "Waiting step over..." << std::endl;
                    waiting_new_step = true;
                human_choice_pub.publish(msg);
            }
        }
        loop.sleep();
    }

    return 0;
}