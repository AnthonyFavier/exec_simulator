#include <ros/ros.h>
#include <termios.h>
#include <vector>

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hmi");
    ros::NodeHandle node_handle;

    valid_actions.push_back("pick 1");
    valid_actions.push_back("pick 2");
    valid_actions.push_back("drink");

    ros::Rate loop(50);
    print_actions();
    while (ros::ok())
    {
        int c = getch(); // call your non-blocking input function
        if (c=='0' || c=='1' || c=='2')
        {
            std::cout << "\nAction " << char(c) << " selected !" << std::endl;
            print_actions();
        }
        else if (c == 'b')
        {
            std::cout << "ROH\r";

        }
        else if (c=='u')
        {
            valid_actions.pop_back();
            valid_actions[1] = "blablater";
            print_actions();
        }

        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}