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


  /* Box 1 */

  moveit_msgs::CollisionObject box_1;
  box_1.id = "box_1";
  box_1.header.frame_id = "world";

  /* Define the primitive and its dimensions. */
  box_1.primitives.resize(1);
  box_1.primitives[0].type = box_1.primitives[0].BOX;
  box_1.primitives[0].dimensions.resize(3);
  box_1.primitives[0].dimensions[0] = 0.35;
  box_1.primitives[0].dimensions[1] = 0.35;
  box_1.primitives[0].dimensions[2] = 0.33;

  /* Define the pose of the object. */
  box_1.primitive_poses.resize(1);  
  box_1.primitive_poses[0].position.x = 0.85;
  box_1.primitive_poses[0].position.y = 0.2;
  box_1.primitive_poses[0].position.z = 0.865;
  box_1.primitive_poses[0].orientation.w = 1.0;

  box_1.operation = box_1.ADD;
  planning_scene_interface.applyCollisionObject(box_1);

  
  /* Box 2 */

  moveit_msgs::CollisionObject box_2;
  box_2.id = "box_2";
  box_2.header.frame_id = "world";

  /* Define the primitive and its dimensions. */
  box_2.primitives.resize(1);
  box_2.primitives[0].type = box_2.primitives[0].BOX;
  box_2.primitives[0].dimensions.resize(3);
  box_2.primitives[0].dimensions[0] = 0.35;
  box_2.primitives[0].dimensions[1] = 0.35;
  box_2.primitives[0].dimensions[2] = 0.33;

  /* Define the pose of the object. */
  box_2.primitive_poses.resize(1);  
  box_2.primitive_poses[0].position.x = 0.85;
  box_2.primitive_poses[0].position.y = -0.20;
  box_2.primitive_poses[0].position.z = 0.865;
  box_2.primitive_poses[0].orientation.w = 1.0;

  box_2.operation = box_2.ADD;
  planning_scene_interface.applyCollisionObject(box_2);


  /* cube */

  moveit_msgs::CollisionObject cube;
  cube.id = "cube";
  cube.header.frame_id = "world";

  /* Define the primitive and its dimensions. */
  cube.primitives.resize(1);
  cube.primitives[0].type = cube.primitives[0].BOX;
  cube.primitives[0].dimensions.resize(3);
  cube.primitives[0].dimensions[0] = 0.12;
  cube.primitives[0].dimensions[1] = 0.12;
  cube.primitives[0].dimensions[2] = 0.12;

  /* Define the pose of the object. */
  cube.primitive_poses.resize(1);  
  cube.primitive_poses[0].position.x = 0.5;
  cube.primitive_poses[0].position.y = -0.5;
  cube.primitive_poses[0].position.z = 0.76;
  cube.primitive_poses[0].orientation.w = 1.0;

  cube.operation = cube.ADD;
  planning_scene_interface.applyCollisionObject(cube);
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
