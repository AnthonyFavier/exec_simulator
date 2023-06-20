// #include <gazebo/math/Rand.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gui/MouseEventHandler.hh>
#include <gazebo/common/MouseEvent.hh>
#include <ros/ros.h> // for acceessing ros
#include <geometry_msgs/Point.h>

namespace gazebo
{
  class MousePosePublisherPlugin : public SystemPlugin
  {
    // Destructor
  public:
    virtual ~MousePosePublisherPlugin()
    {
      gui::MouseEventHandler::Instance()->RemovePressFilter("glwidget");
    }

    // Called after the plugin has been constructed.
  public:
    void Load(int /*_argc*/, char ** /*_argv*/)
    {
      gui::MouseEventHandler::Instance()->AddPressFilter("glwidget",
                                                         boost::bind(&MousePosePublisherPlugin::OnMousePress, this, _1));

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

      this->mouse_pub = this->rosNode->advertise<geometry_msgs::Point>("/mouse_pressed_pose", 10);
      
      ROS_INFO("Plugin: Mouse Pose Publisher Loaded !");
    }

  public:
    bool OnMousePress(const common::MouseEvent &_event)
    {
      std::cout << "Mouse press detected: ";
      std::cout << _event.Pos().X() << "," << _event.Pos().Y() << std::endl;
      geometry_msgs::Point msg;
      msg.x = _event.Pos().X();
      msg.y = _event.Pos().Y();
      mouse_pub.publish(msg);
      return true;
    }

  private:
    std::unique_ptr<ros::NodeHandle> rosNode;
    ros::Publisher mouse_pub;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_SYSTEM_PLUGIN(MousePosePublisherPlugin)
}