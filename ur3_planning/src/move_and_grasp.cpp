#include <ros/ros.h>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "movit_control");
  ros::NodeHandle n("~");
  
  // Start a ROS spinning thread (required for processing callbacks while moveit is blocking)
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  moveit::planning_interface::MoveGroup group("manipulator");  
  moveit::planning_interface::MoveGroup gripper("gripper");  
  
  group.setPoseReferenceFrame("base_link"); // plan w.r.t. base_link frame by default
//   group.setEndEffectorLink();
  
  ros::Publisher display_publisher = n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  ROS_INFO("Endeffector frame: %s", group.getEndEffectorLink().c_str());
  
  // Set values for your gripper here
  std::vector<double> gripper_open;
  gripper_open.push_back(0.3);
  gripper_open.push_back(0.3);
  std::vector<double> gripper_closed;
  gripper_closed.push_back(0.01);
  gripper_closed.push_back(0.01);

  group.setGoalPositionTolerance(0.04);
  group.setGoalOrientationTolerance(0.1);
  group.allowReplanning(true);
    
                
  // test gripper
  ROS_INFO("Open Gripper");
  gripper.setJointValueTarget(gripper_open);
  gripper.move();      
  
  ros::Duration(5).sleep(); // wait a few seconds here
  

  // Planning to a joint-space goal  
  ROS_INFO("Planing to a joint-space goal");
    
  // Get current joint values
  std::vector<double> group_variable_values;
  group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);

  // Set a new target
  group_variable_values[0] += -1; // rotate a few radiant
  group_variable_values[1] += M_PI/2; // here as well
  group.setJointValueTarget(group_variable_values);
  group.move(); // we do not test before, just move if possible ;-)
                // move is a blocking call

  ros::Duration(2).sleep(); // stay 2 seconds at this goal
  // now back to default
  ROS_INFO("And back to default conf...");
  
  std::fill(group_variable_values.begin(), group_variable_values.end(), 0.0);
  group.setJointValueTarget(group_variable_values);
  group.move(); // we do not test before, just move if possible ;-)
  
  ros::Duration(5).sleep(); // stay 2 seconds at this goal
  // Planning to a task-space end effector goal
  /*ROS_INFO("Planing to a task-space goal");  
    
  moveit::planning_interface::MoveGroup::Plan my_plan;
  geometry_msgs::Pose target_pose2;
  target_pose2.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
  target_pose2.position.x = 0.3;
  target_pose2.position.y = 0.2;
  target_pose2.position.z = 1.0;
  group.setPoseTarget(target_pose2);
  bool success = false;
  success = group.plan(my_plan);
 
  if (success)
  {
      group.asyncExecute(my_plan);
  }
  else
  {
      ROS_ERROR("Cannot find a valid plan...");
  }
  
  ros::Duration(5).sleep();*/ // wait a few seconds here   
  
  ROS_INFO("Close Gripper");
  gripper.setJointValueTarget(gripper_closed);
  gripper.move();              
  ros::Duration(3).sleep();              
  
  ROS_INFO("Demo completed. Waiting for user shutdown (ctrl-c).");
  ros::waitForShutdown();

  return 0;
}