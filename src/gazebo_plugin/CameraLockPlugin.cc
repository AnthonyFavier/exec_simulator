#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/rendering/rendering.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>
#include <cstdlib>
#include <std_msgs/Empty.h>
#include <gazebo/common/Event.hh>
#include <ros/ros.h> // for acceessing ros

namespace gazebo
{
    class CameraLockPlugin : public VisualPlugin
    {
    public:
        void Load(
            gazebo::rendering::VisualPtr visual, sdf::ElementPtr sdf)
        {
            // get visual scene
            gazebo::rendering::ScenePtr scene = visual->GetScene();
            m_camera = scene->GetUserCamera(0);

            m_update_connection = gazebo::event::Events::ConnectPreRender(
                boost::bind(&CameraLockPlugin::onUpdate, this));
         
            ROS_INFO("Plugin: Camera Lock Loaded !");
        }

    public:
        void onUpdate()
        {
            // publish pose
            ignition::math::Pose3d custom_pose = ignition::math::Pose3d(
                2.24, 0.0, 2.49, 0, 0.86, 3.14);

            // set pose
            m_camera->SetWorldPose(custom_pose);
        }

    private:
        rendering::UserCameraPtr m_camera;
        event::ConnectionPtr m_update_connection;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_VISUAL_PLUGIN(CameraLockPlugin)
};