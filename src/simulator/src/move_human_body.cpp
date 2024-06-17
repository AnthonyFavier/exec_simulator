#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/String.h>
#include <algorithm>
#include "sim_msgs/MoveArm.h"
#include "moveit_msgs/Constraints.h"
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"
#include "gazebo_msgs/SetModelState.h"
#include "moveit_msgs/RobotState.h"
#include "moveit/robot_state/robot_state.h"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <Eigen/Dense>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ros::ServiceClient set_model_state_client;
geometry_msgs::Pose current_human_pose;
double init_x, init_yaw;

enum STATE{ TURN_180_LEFT,  TURN_180_RIGHT,
            TURN_90_LEFT,   TURN_90_RIGHT,
            MOVE_FORWARD,
            STATIC,
            RESET
            };
STATE state = STATIC;

geometry_msgs::Point make_point(double x, double y, double z)
{
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

geometry_msgs::Quaternion make_quaternion(double x /*= 0.0*/, double y /*= 0.0*/, double z /*= 0.0*/, double w /*= 1.0*/)
{
    geometry_msgs::Quaternion q;
    q.x = x;
    q.y = y;
    q.z = z;
    q.w = w;
    return q;
}

geometry_msgs::Quaternion make_quaternion_RPY(double r /*= 0.0*/, double p /*= 0.0*/, double y /*= 0.0*/)
{
    tf2::Quaternion my_q;
    my_q.setRPY(r, p, y);

    geometry_msgs::Quaternion q;
    q.x = my_q.getX();
    q.y = my_q.getY();
    q.z = my_q.getZ();
    q.w = my_q.getW();
    return q;
}

geometry_msgs::Pose make_pose(geometry_msgs::Point p, geometry_msgs::Quaternion q)
{
    geometry_msgs::Pose pose;
    pose.position = p;
    pose.orientation = q;
    return pose;
}

void move_human_cb(std_msgs::Int32 msg)
{
	switch (msg.data)
	{
		// TURN
		case 0:{
			tf::Quaternion q(
			current_human_pose.orientation.x,
			current_human_pose.orientation.y,
			current_human_pose.orientation.z,
			current_human_pose.orientation.w);

			tf::Matrix3x3 m(q);
			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);

			init_yaw = yaw;
			state = STATE::TURN_180_LEFT;

			ROS_WARN("TURN human body: init_yaw=%f", init_yaw);
			break;}

		// MOVE
		case 1:
			init_x = current_human_pose.position.x;
			state = STATE::MOVE_FORWARD;
			ROS_WARN("MOVE human body: init_x=%f", init_x);
			break;

		// RESET
		case -1:
			state = STATE::RESET;
			ROS_WARN("RESET human body");
			break;
		
		default:
			break;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_hand");
	ros::NodeHandle node_handle;

	ros::AsyncSpinner spinner(2);
	spinner.start();

	geometry_msgs::Pose init_human_pose = make_pose( make_point(2.6,0,0), make_quaternion_RPY(0,0,M_PI) );
	current_human_pose = init_human_pose;

	set_model_state_client = node_handle.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
	ros::Subscriber move_human_sub = node_handle.subscribe("/move_human_body", 1, move_human_cb);

	gazebo_msgs::SetModelState srv_set;
	srv_set.request.model_state.model_name = "human_body";

	ros::Rate loop(30);
	while (ros::ok())
	{
		tf::Quaternion q(
			current_human_pose.orientation.x,
			current_human_pose.orientation.y,
			current_human_pose.orientation.z,
			current_human_pose.orientation.w);

		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

		switch(state)
		{
			case RESET:
				current_human_pose = init_human_pose;
				break;

			case STATIC:
				break;

			case TURN_180_LEFT:{
				tf2::Quaternion q2;
				yaw+=0.03;
				if(yaw>M_PI)
					yaw -= 2*M_PI;
				q2.setRPY(roll, pitch, yaw);

				ROS_WARN("yaw = %f", yaw);

				current_human_pose.orientation = tf2::toMsg(q2);

				// End condition
				double a = init_yaw;
				double c = yaw;

				bool over = false;
				if(a >= 0 && (a-c<M_PI || (c>0 && c<a)))
					over = true;
				if(a < 0 && (a-c<-M_PI || (c<0 && c<a)))
					over = true;
				if(over)
				{
					state = STATE::STATIC;
					q2.setRPY(0,0,init_yaw+M_PI);
					current_human_pose.orientation = tf2::toMsg(q2);
				}

				break;}

			case MOVE_FORWARD:{
				double delta_x = 2.0;

				double c = current_human_pose.position.x;
				bool direction_positive = yaw<0.01 && yaw>-0.01;

				if(direction_positive)
				{
					current_human_pose.position.x += 0.01;

					if(c>=init_x+delta_x)
					{
						state = STATE::STATIC;
						current_human_pose.position.x =  init_x+delta_x;
					}
				}
				else
				{
					current_human_pose.position.x -= 0.01;

					if(c<=init_x-delta_x)
					{
						state = STATE::STATIC;
						current_human_pose.position.x = init_x-delta_x;
					}
				}
				break;}


			default:
				break;
		}

		srv_set.request.model_state.pose = current_human_pose;
		set_model_state_client.call(srv_set);

		loop.sleep();
	}

	ros::shutdown();
	return 0;
}
