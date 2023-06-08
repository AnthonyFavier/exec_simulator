/*
 * Copyright (C) 2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <sstream>
#include <gazebo/msgs/msgs.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/gui/MouseEventHandler.hh>

// #include <gazebo/math/Rand.hh>
// #include <gazebo/gui/GuiIface.hh>
// #include <gazebo/rendering/rendering.hh>
// #include <gazebo/gazebo.hh>
// #include <gazebo/gui/MouseEventHandler.hh>
// #include <gazebo/common/MouseEvent.hh>

using namespace gazebo;

class GAZEBO_VISIBLE TestPlugin : public GUIPlugin
{
  public: TestPlugin()
  {
    std::cout << "SALUT\n";
    // gui::MouseEventHandler::Instance()->AddPressFilter("glwidget", std::bind(&TestPlugin::OnMousePress, this, std::placeholders::_1));
  }

  public: virtual ~TestPlugin()
  {
    // gui::MouseEventHandler::Instance()->RemovePressFilter("glwidget");
  }

  public: bool OnMousePress(const common::MouseEvent &_event)
  {
    std::cout << "Mouse press detected!\n";
    return true;
  }
};

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(TestPlugin)