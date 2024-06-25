#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

void init_scene_epistemic()
{
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  /* TABLE */

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



  /* Boxes */
  moveit_msgs::CollisionObject box_1, box_2, box_3;
  box_1.operation = box_1.ADD;
  box_1.header.frame_id = "world";

  /* Define the primitive and its dimensions. */
  box_1.primitives.resize(1);
  box_1.primitives[0].type = box_1.primitives[0].BOX;
  box_1.primitives[0].dimensions.resize(3);
  box_1.primitives[0].dimensions[0] = 0.53;
  box_1.primitives[0].dimensions[1] = 0.53;
  box_1.primitives[0].dimensions[2] = 0.38;

  /* Define the pose of the object. */
  box_1.primitive_poses.resize(1);
  box_1.primitive_poses[0].position.x = 0.85;
  box_1.primitive_poses[0].position.y = -0.25;
  box_1.primitive_poses[0].position.z = 0.89;
  box_1.primitive_poses[0].orientation.w = 1.0;

  box_1.id = "box_1";
  planning_scene_interface.applyCollisionObject(box_1);

  box_2 = box_1;
  box_2.id = "box_2";
  box_2.primitive_poses[0].position.x = 0.85;
  box_2.primitive_poses[0].position.y = 0.1;
  box_2.primitive_poses[0].position.z = 0.89;
  planning_scene_interface.applyCollisionObject(box_2);

  box_3 = box_1;
  box_3.id = "box_3";
  box_3.primitive_poses[0].position.x = 0.85;
  box_3.primitive_poses[0].position.y = 0.45;
  box_3.primitive_poses[0].position.z = 0.89;
  planning_scene_interface.applyCollisionObject(box_3);


  /* cube r1 */

  moveit_msgs::CollisionObject r1;
  r1.id = "r1";
  r1.header.frame_id = "world";

  /* Define the primitive and its dimensions. */
  r1.primitives.resize(1);
  r1.primitives[0].type = r1.primitives[0].BOX;
  r1.primitives[0].dimensions.resize(3);
  r1.primitives[0].dimensions[0] = 0.10;
  r1.primitives[0].dimensions[1] = 0.10;
  r1.primitives[0].dimensions[2] = 0.10;

  /* Define the pose of the object. */
  r1.primitive_poses.resize(1);  
  r1.primitive_poses[0].position.x = 0.5;
  r1.primitive_poses[0].position.y = -0.6;
  r1.primitive_poses[0].position.z = 0.75;
  r1.primitive_poses[0].orientation.w = 1.0;

  r1.operation = r1.ADD;
  planning_scene_interface.applyCollisionObject(r1);
}

void init_scene_stack_empiler_2()
{
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  /* TABLE */

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
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::string scenario_name = argv[1];

  if(scenario_name=="stack_empiler_2")
    init_scene_stack_empiler_2();
  else if(scenario_name=="epistemic")
    init_scene_epistemic();

  ros::shutdown();
  return 0;
}
