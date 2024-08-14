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
  collision_object_1.primitives[0].dimensions[1] = 1.91;
  collision_object_1.primitives[0].dimensions[2] = 0.7;

  /* Define the pose of the table. */
  collision_object_1.primitive_poses.resize(1);  
  collision_object_1.primitive_poses[0].position.x = 0.85;
  collision_object_1.primitive_poses[0].position.y = -0.07;
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
  box_1.primitives[0].dimensions[0] = 0.50;
  box_1.primitives[0].dimensions[1] = 0.50;
  box_1.primitives[0].dimensions[2] = 0.37;

  /* Define the pose of the object. */
  box_1.primitive_poses.resize(1);
  box_1.primitive_poses[0].position.x = 0.85;
  box_1.primitive_poses[0].position.y = -0.07;
  box_1.primitive_poses[0].position.z = 0.885;
  box_1.primitive_poses[0].orientation.w = 1.0;

  box_1.id = "box_1";
  planning_scene_interface.applyCollisionObject(box_1);

  box_2 = box_1;
  box_2.id = "box_2";
  box_2.primitive_poses[0].position.x = 0.85;
  box_2.primitive_poses[0].position.y = 0.24;
  box_2.primitive_poses[0].position.z = 0.885;
  planning_scene_interface.applyCollisionObject(box_2);

  box_3 = box_1;
  box_3.id = "box_3";
  box_3.primitive_poses[0].position.x = 0.85;
  box_3.primitive_poses[0].position.y = 0.55;
  box_3.primitive_poses[0].position.z = 0.885;
  planning_scene_interface.applyCollisionObject(box_3);


  /* cube r1 */

  moveit_msgs::CollisionObject r1;
  r1.id = "r1";
  r1.header.frame_id = "world";

  /* Define the primitive and its dimensions. */
  r1.primitives.resize(1);
  r1.primitives[0].type = r1.primitives[0].BOX;
  r1.primitives[0].dimensions.resize(3);
  r1.primitives[0].dimensions[0] = 0.12;
  r1.primitives[0].dimensions[1] = 0.12;
  r1.primitives[0].dimensions[2] = 0.12;

  /* Define the pose of the object. */
  r1.primitive_poses.resize(1);  
  r1.primitive_poses[0].position.x = 0.5;
  r1.primitive_poses[0].position.y = -0.4;
  r1.primitive_poses[0].position.z = 0.75;
  r1.primitive_poses[0].orientation.w = 1.0;

  r1.operation = r1.ADD;
  planning_scene_interface.applyCollisionObject(r1);

  /* cube y1 */

  moveit_msgs::CollisionObject y1;
  y1.id = "y1";
  y1.header.frame_id = "world";

  /* Define the primitive and its dimensions. */
  y1.primitives.resize(1);
  y1.primitives[0].type = y1.primitives[0].BOX;
  y1.primitives[0].dimensions.resize(3);
  y1.primitives[0].dimensions[0] = 0.12;
  y1.primitives[0].dimensions[1] = 0.12;
  y1.primitives[0].dimensions[2] = 0.12;

  /* Define the pose of the object. */
  y1.primitive_poses.resize(1);  
  y1.primitive_poses[0].position.x = 0.5;
  y1.primitive_poses[0].position.y = -0.65;
  y1.primitive_poses[0].position.z = 0.75;
  y1.primitive_poses[0].orientation.w = 1.0;

  y1.operation = y1.ADD;
  planning_scene_interface.applyCollisionObject(y1);


  /* cube g2 */

  moveit_msgs::CollisionObject g2;
  g2.id = "g2";
  g2.header.frame_id = "world";

  /* Define the primitive and its dimensions. */
  g2.primitives.resize(1);
  g2.primitives[0].type = g2.primitives[0].BOX;
  g2.primitives[0].dimensions.resize(3);
  g2.primitives[0].dimensions[0] = 0.12;
  g2.primitives[0].dimensions[1] = 0.12;
  g2.primitives[0].dimensions[2] = 0.12;

  /* Define the pose of the object. */
  g2.primitive_poses.resize(1);  
  g2.primitive_poses[0].position.x = 0.5;
  g2.primitive_poses[0].position.y = -0.65;
  g2.primitive_poses[0].position.z = 0.85;
  g2.primitive_poses[0].orientation.w = 1.0;

  g2.operation = g2.ADD;
  planning_scene_interface.applyCollisionObject(g2);

  /* cube g1 */

  moveit_msgs::CollisionObject g1;
  g1.id = "g1";
  g1.header.frame_id = "world";

  /* Define the primitive and its dimensions. */
  g1.primitives.resize(1);
  g1.primitives[0].type = g1.primitives[0].BOX;
  g1.primitives[0].dimensions.resize(3);
  g1.primitives[0].dimensions[0] = 0.12;
  g1.primitives[0].dimensions[1] = 0.12;
  g1.primitives[0].dimensions[2] = 0.12;

  /* Define the pose of the object. */
  g1.primitive_poses.resize(1);  
  g1.primitive_poses[0].position.x = 0.85;
  g1.primitive_poses[0].position.y = -0.45;
  g1.primitive_poses[0].position.z = 0.75;
  g1.primitive_poses[0].orientation.w = 1.0;

  g1.operation = g1.ADD;
  planning_scene_interface.applyCollisionObject(g1);
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
