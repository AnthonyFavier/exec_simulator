#include <std_msgs/String.h>
// #include <gazebo/rendering/rendering.hh>
#include <gazebo/gazebo.hh>
// #include <gazebo/common/MouseEvent.hh>
#include <ros/ros.h> // for acceessing ros
#include <gazebo/common/Plugin.hh>
// #include <gazebo/gui/GuiIface.hh>
#include <gazebo/gui/GuiPlugin.hh>
// #include <geometry_msgs/Point.h>
// #include <sstream>
// #include <gazebo/msgs/msgs.hh>
// #include <gazebo/transport/transport.hh>

namespace gazebo
{
  class GUIZones : public GUIPlugin
  {

    // Constructor
  public:
    GUIZones()
    {
      // chech if ros is initized or not
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        // good pratice of init ros node
        ros::init(argc, argv, "gazebo_client",
                  ros::init_options::NoSigintHandler);
      }

      // Create our ROS node. This acts in a similar manner to gazebo node
      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

      // this->mouse_pub = this->rosNode->advertise<geometry_msgs::Point>("/mouse_pressed_pose", 10);
      this->text_sub = this->rosNode->subscribe("/text_gazebo_label", 10, &GUIZones::OnMsg, this);

      /////////////////////////////

      // Set the frame background and foreground colors
      this->setStyleSheet("QFrame { background-color : rgba(255, 255, 0, 20); color : white; }");
      // this->setStyleSheet("background : transparent;");
      this->setAttribute(Qt::WA_TranslucentBackground, true);

      QHBoxLayout *mainLayout = new QHBoxLayout;
      QFrame *mainFrame = new QFrame();

      // Add the frame to the main layout
      mainLayout->addWidget(mainFrame);

      // Remove margins to reduce space
      mainLayout->setContentsMargins(0, 0, 0, 0);

      this->setLayout(mainLayout);

      // Position and resize this widget
      this->move(100, 100);
      this->resize(300, 100);

      ROS_INFO("Plugin: GUI Zones loaded !");
    }

    // Destructor
  public:
    ~GUIZones()
    {
    }

  public:
    void OnMsg(const std_msgs::String &_msg)
    {
    }

  private:
    std::unique_ptr<ros::NodeHandle> rosNode;
    ros::Subscriber text_sub;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_GUI_PLUGIN(GUIZones)
}