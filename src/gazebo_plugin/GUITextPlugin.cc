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
  class GUITextPlugin : public GUIPlugin
  {

    // Constructor
  public:
    GUITextPlugin()
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
      this->text_sub = this->rosNode->subscribe("/text_gazebo_label", 10, &GUITextPlugin::OnMsg, this);

      /////////////////////////////

      // Set the frame background and foreground colors
      this->setStyleSheet(
          "QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

      // Create the main layout
      QHBoxLayout *mainLayout = new QHBoxLayout;

      // Create the frame to hold all the widgets
      QFrame *mainFrame = new QFrame();

      // Create the layout that sits inside the frame
      QVBoxLayout *frameLayout = new QVBoxLayout();

      this->label = new QLabel("Robot status\ntext prompt");
      this->label->setAlignment(Qt::AlignCenter);
      // this->label->setFont(QFont("Arial", 22));
      this->label->setFont(QFont("Arial", 16));
      frameLayout->addWidget(label, Qt::AlignCenter);

      // Add frameLayout to the frame
      mainFrame->setLayout(frameLayout);

      // Add the frame to the main layout
      mainLayout->addWidget(mainFrame);

      // Remove margins to reduce space
      frameLayout->setContentsMargins(0, 0, 0, 0);
      mainLayout->setContentsMargins(0, 0, 0, 0);

      // OUTER
      // QHBoxLayout *outerLayout = new QHBoxLayout;
      // outerLayout->setContentsMargins(0, 0, 0, 0);
      // QFrame *outerFrame = new QFrame();
      // outerFrame->setLayout(outerLayout);
      // outer



      this->setLayout(mainLayout);

      // Position and resize this widget
      // this->move(1150, 300);
      this->move(20, 20);
      this->resize(400, 100);

      ROS_INFO("Plugin: GUI Text loaded !");
    }

    // Destructor
  public:
    ~GUITextPlugin()
    {
    }

  public:
    void OnMsg(const std_msgs::String &_msg)
    {
      this->label->setText(_msg.data.c_str());
    }

  private:
    std::unique_ptr<ros::NodeHandle> rosNode;
    ros::Subscriber text_sub;
    QLabel *label;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_GUI_PLUGIN(GUITextPlugin)
}