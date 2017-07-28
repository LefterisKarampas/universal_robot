#include <ros/ros.h>
#include <iostream>
#include <string>
#include <stdlib.h>     /* atof */
#include <unistd.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h> 
#include <moveit/trajectory_processing/iterative_time_parameterization.h>


using namespace std;

int main(int argc, char **argv) {
    //cout << argv[0] << endl;
    ros::init(argc, argv,"moveit_group_goal");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nodeHandle;

    moveit::planning_interface::MoveGroup group("manipulator");
    group.setMaxVelocityScalingFactor(0.1);
    group.setMaxAccelerationScalingFactor(0.1);
    group.setPlanningTime(10.0);
    group.setNumPlanningAttempts(100);
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setPoseReferenceFrame("world");
    group.setGoalTolerance(0.1);


  
  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose target_pose;
  target_pose.position.x = atof(argv[1]);
  target_pose.position.y = atof(argv[2]);
  target_pose.position.z = atof(argv[3]);
  waypoints.push_back(target_pose);
  geometry_msgs::Pose target_pose1;
  target_pose1.position.x = atof(argv[4]);
  target_pose1.position.y = atof(argv[5]);
  target_pose1.position.z = atof(argv[6]);
  waypoints.push_back(target_pose1);
  //waypoints.push_back(target_pose2);
  moveit_msgs::RobotTrajectory trajectory_msg;
  double fraction = group.computeCartesianPath(waypoints,
                                               0.01,  // eef_step
                                               0.0,   // jump_threshold
                                               trajectory_msg, true);
  // The trajectory needs to be modified so it will include velocities as well.
  // First to create a RobotTrajectory object
  robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "manipulator");

  // Second get a RobotTrajectory from trajectory
  rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory_msg);
 
  // Thrid create a IterativeParabolicTimeParameterization object
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  bool success;
  // Fourth compute computeTimeStamps
  success = iptp.computeTimeStamps(rt);
  ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

  // Get RobotTrajectory_msg from RobotTrajectory
  rt.getRobotTrajectoryMsg(trajectory_msg);
  // Finally plan and execute the trajectory
  moveit::planning_interface::MoveGroup::Plan plan;
  plan.trajectory_ = trajectory_msg;
  //group.setPoseTargets(waypoints);
  //success = group.plan(plan);
  ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",fraction * 100.0);  
  ROS_INFO("plan: %s",success?"SUCCESS":"FAILED");
  if(success) {
    char q;
    std::cout << "Please make sure that your robot can move freely between these poses before proceeding!\n";
    std::cout << "Continue? y/n: ";
    std::cin >> q;
    std::cout << endl;
    if(q == 'y'){
       ROS_INFO("Moving...");
        group.asyncExecute(plan);
    }
  sleep(5.0);
  } 
  return 0;
}