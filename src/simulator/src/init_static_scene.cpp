#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  moveit_msgs::CollisionObject collision_object_1;
  collision_object_1.id = "table1";
  collision_object_1.header.frame_id = "world";

  /* Define the primitive and its dimensions. */
  collision_object_1.primitives.resize(1);
  collision_object_1.primitives[0].type = collision_object_1.primitives[0].BOX;
  collision_object_1.primitives[0].dimensions.resize(3);
  collision_object_1.primitives[0].dimensions[0] = 0.969;
  collision_object_1.primitives[0].dimensions[1] = 1.59;
  collision_object_1.primitives[0].dimensions[2] = 0.7;

  /* Define the pose of the table. */
  collision_object_1.primitive_poses.resize(1);  
  collision_object_1.primitive_poses[0].position.x = 0.85;
  collision_object_1.primitive_poses[0].position.y = 0.0;
  collision_object_1.primitive_poses[0].position.z = collision_object_1.primitives[0].dimensions[2]/2;
  collision_object_1.primitive_poses[0].orientation.w = 1.0;

  collision_object_1.operation = collision_object_1.ADD;

  planning_scene_interface.applyCollisionObject(collision_object_1);

  ros::shutdown();
  return 0;
}
