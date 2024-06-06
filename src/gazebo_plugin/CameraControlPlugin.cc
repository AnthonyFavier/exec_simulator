#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/rendering/rendering.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>
#include <cstdlib>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <gazebo/common/Event.hh>
#include <ros/ros.h> // for acceessing ros
#include <gazebo/gui/GuiIface.hh>

#include <thread>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

enum STATE{ TURN_180_LEFT,  TURN_180_RIGHT,
            TURN_90_LEFT,   TURN_90_RIGHT,
            MOVE_FORWARD,
            STATIC
            };

namespace gazebo
{
    class CameraControlPlugin : public VisualPlugin
    {
    public:
        void Load(
            gazebo::rendering::VisualPtr visual, sdf::ElementPtr sdf)
        {
            // get visual scene
            gazebo::rendering::ScenePtr scene = visual->GetScene();
            m_camera = scene->GetUserCamera(0);

            m_update_connection = gazebo::event::Events::ConnectPreRender(
                boost::bind(&CameraControlPlugin::onUpdate, this));

            // check if ros is initized or not
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

            this->m_control_sub = this->rosNode->subscribe("/control_camera", 1, &CameraControlPlugin::control_gazebo_camera_cb, this);
            this->rosQueueThread = std::thread(std::bind(&CameraControlPlugin::QueueThread, this));

            m_current_pose = ignition::math::Pose3d(2.6, 0, 2.0, 0, 0.56, M_PI);
            m_rotation_incr = 0.03;
            m_move_incr = 0.01;

            ROS_INFO("Plugin: Camera Control Loaded !");
        }

        ~CameraControlPlugin()
        {
            if (this->rosQueueThread.joinable())
            {
            this->rosQueueThread.join();
            }
        }

        void QueueThread()
        {
            static const double timeout = 0.01;

            while (this->rosNode->ok())
            {
            this->rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }

        void control_gazebo_camera_cb(std_msgs::Int32 msg)
        {
            ROS_INFO("plop test callback Camera Control Plugin");

            // Receive command, set state

            switch(msg.data)
            {
                // TURN_180_LEFT
                case 0:
                    m_init_yaw = m_current_pose.Yaw();
                    m_camera_state = STATE::TURN_180_LEFT;
                    ROS_WARN("CAMERA STATE SWITCH TO TURN_180_LEFT");
                    break;

                // MOVE_FORWARD
                case 1:
                    m_init_x = m_current_pose.X();
                    m_camera_state = STATE::MOVE_FORWARD;
                    ROS_WARN("CAMERA STATE SWITCH TO MOVE_FORWARD");
                    break;

                default:
                    break;
            }

        }

    public:
        void onUpdate()
        {
            // // Check state, and update current_pose accordingly
            // // possibly update state back to nominal (static)
            // // check frequency update

            switch(m_camera_state)
            {
                case STATIC:
                    // ROS_INFO_STREAM("1 Yaw = " << m_current_pose.Yaw());
                    break;

                case TURN_180_LEFT:{
                    double delta_angle = M_PI;

                    m_current_pose.Set( m_current_pose.X(), m_current_pose.Y(), m_current_pose.Z(),
                                        m_current_pose.Roll(), m_current_pose.Pitch(), m_current_pose.Yaw()+m_rotation_incr);

                    // ROS_INFO_STREAM("2 Yaw = " << m_current_pose.Yaw());

                    double a = m_init_yaw;
                    double c = m_current_pose.Yaw();

                    bool over = false;
                    if(a >= 0 && (a-c<delta_angle || (c>0 && c<a)))
                        over = true;
                    if(a < 0 && (a-c<-delta_angle || (c<0 && c<a)))
                        over = true;

                    if(over)
                    {
                        m_camera_state = STATE::STATIC;
                        m_current_pose.Set( m_current_pose.X(), m_current_pose.Y(), m_current_pose.Z(),
                                        m_current_pose.Roll(), m_current_pose.Pitch(), m_init_yaw+delta_angle);
                    }

                    break;}

                

                case MOVE_FORWARD:{

                    double delta_x = 1.5;

                    // ROS_INFO_STREAM("2 X = " << m_current_pose.X());

                    double c = m_current_pose.X();
                    bool direction_positive = m_current_pose.Yaw()<0.01 && m_current_pose.Yaw()>-0.01;

                    if(direction_positive)
                    {
                        m_current_pose.Set( m_current_pose.X()+m_move_incr, m_current_pose.Y(), m_current_pose.Z(),
                                        m_current_pose.Roll(), m_current_pose.Pitch(), m_current_pose.Yaw());

                        if(c>=m_init_x+delta_x)
                        {
                            m_camera_state = STATE::STATIC;
                            m_current_pose.Set( m_init_x+delta_x, m_current_pose.Y(), m_current_pose.Z(),
                                            m_current_pose.Roll(), m_current_pose.Pitch(), m_current_pose.Yaw());
                        }
                    }
                    else
                    {
                        m_current_pose.Set( m_current_pose.X()-m_move_incr, m_current_pose.Y(), m_current_pose.Z(),
                                        m_current_pose.Roll(), m_current_pose.Pitch(), m_current_pose.Yaw());

                        if(c<=m_init_x-delta_x)
                        {
                            m_camera_state = STATE::STATIC;
                            m_current_pose.Set( m_init_x-delta_x, m_current_pose.Y(), m_current_pose.Z(),
                                            m_current_pose.Roll(), m_current_pose.Pitch(), m_current_pose.Yaw());
                        }
                    }
                    break;}

                
                default:
                    ROS_INFO("3");
                    break;
            }

            // set pose
            if(m_camera != NULL)
            {
                if(m_current_pose.Yaw()==-M_PI)
                    m_current_pose.Set( m_current_pose.X(), m_current_pose.Y(), m_current_pose.Z(), m_current_pose.Roll(), m_current_pose.Pitch(), M_PI);

                m_camera->SetWorldPose(m_current_pose);
            }

            ros::spinOnce();
        }

    private:
        rendering::UserCameraPtr m_camera;
        event::ConnectionPtr m_update_connection;

        ignition::math::Pose3d m_current_pose;
        STATE m_camera_state = STATE::STATIC;
        float m_rotation_incr;
        float m_move_incr; 
        double m_init_yaw;
        double m_init_x;

        std::unique_ptr<ros::NodeHandle> rosNode;
        ros::CallbackQueue rosQueue;
        std::thread rosQueueThread;
        ros::Subscriber m_control_sub;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_VISUAL_PLUGIN(CameraControlPlugin)
};