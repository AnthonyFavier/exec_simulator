#include <gazebo/math/Rand.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gui/MouseEventHandler.hh>
#include <gazebo/common/MouseEvent.hh>


namespace gazebo
{
  class SystemGUI : public SystemPlugin
  {

    // Destructor
    public: virtual ~SystemGUI()
    {
        gui::MouseEventHandler::Instance()->RemovePressFilter("glwidget");
    }

    // Called after the plugin has been constructed.
    public: void Load(int /*_argc*/, char ** /*_argv*/)
    {   
        gui::MouseEventHandler::Instance()->AddPressFilter("glwidget",
            boost::bind(&SystemGUI::OnMousePress, this, _1));
    }

    public: bool OnMousePress(const common::MouseEvent & /*_event*/)
    {
        std::cout << "Mouse press detected!\n";
        return true;
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_SYSTEM_PLUGIN(SystemGUI)
}