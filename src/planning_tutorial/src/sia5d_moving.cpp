#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sia5d_arm_testing");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();


  /* This sleep is ONLY to allow Rviz to come up */
  sleep(6.0);
  ROS_INFO("Running sia5d triangular movement");
  
  // The :move_group_interface:`MoveGroup` class can be easily 
  // setup using just the name
  // of the group you would like to control and plan for.
  moveit::planning_interface::MoveGroup group("sia5");

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to deal directly with the world.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

  // (Optional) Create a publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());
  
  geometry_msgs::Pose start_pose;
  start_pose.orientation.w = 0;
  start_pose.orientation.x = 0.707;
  start_pose.orientation.y = 0;
  start_pose.orientation.z = 0.707;
  start_pose.position.x = 0.25;
  start_pose.position.y = -0.1;
  start_pose.position.z = 0.60;    
  
  group.setPoseTarget(start_pose);
  
  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);
  ROS_INFO("Moving to start position %s",success?"":"FAILED");
  
  group.move();
  
  sleep(3.0);
  
  
  std::vector<geometry_msgs::Pose> waypoints;
  
  geometry_msgs::Pose target_pose = start_pose;
  
  target_pose.position.y += 0.1;
  target_pose.position.z += 0.17;
  waypoints.push_back(target_pose);

  target_pose.position.z -= 0.17;
  target_pose.position.y += 0.1;
  waypoints.push_back(target_pose); 

  target_pose.position.y -= 0.2;
  waypoints.push_back(target_pose);  // back to start

  // We want the cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively
  // disabling it.
  moveit_msgs::RobotTrajectory trajectory;
  double fraction = group.computeCartesianPath(waypoints,
                                               0.01,  // eef_step
                                               0.0,   // jump_threshold
                                               trajectory);
  
  ROS_INFO("Visualizing cartesian plan (%.2f%% acheived)",
        fraction * 100.0); 
  
  my_plan.trajectory_ = trajectory;
  
  group.execute(my_plan);
  sleep(5);  
  ros::shutdown();  
return 0;
}