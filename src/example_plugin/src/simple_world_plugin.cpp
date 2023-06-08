#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <gazebo/gui/MouseEventHandler.hh>
// #include <gazebo/gui/GuiPlugin.hh>

namespace gazebo
{
class WorldPluginTutorial : public WorldPlugin
{
public:
  WorldPluginTutorial() : WorldPlugin()
  {
  }
 
  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
  {
    // Make sure the ROS node for Gazebo has already been initialized                                                                                   
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
 
    ROS_WARN("Hello World!");
    // gui::MouseEventHandler::Instance()->AddPressFilter("glwidget", std::bind(&WorldPluginTutorial::OnMousePress, this, std::placeholders::_1));
    gui::MouseEventHandler::Instance()->AddReleaseFilter("test", std::bind(&WorldPluginTutorial::OnMouseRelease, this, std::placeholders::_1));
  }

  bool OnMousePress(const common::MouseEvent &_event)
  {
    ROS_WARN("FOZANFOZ");
    return true;
  }

  bool OnMouseRelease(const common::MouseEvent &_event)
  {
    ROS_WARN("FOZANffffFOZ");
    return true;
  }
 
};
GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}