#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

  planning_pipeline::PlanningPipelinePtr planning_pipeline(
      new planning_pipeline::PlanningPipeline(robot_model, node_handle, "planning_plugin", "request_adapters"));

  /* Sleep a little to allow time to startup rviz, etc. */
  ros::WallDuration sleep_time(5.0);
  sleep_time.sleep();

  // Pose Goal
  // ^^^^^^^^^
  // We will now create a motion plan request for the right arm of the PR2
  // specifying the desired pose of the end-effector as input.
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "wrist_3_link";
  pose.pose.position.x = 0.0;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.5;
  pose.pose.orientation.w = 1.0;

  // A tolerance of 0.01 m is specified in position
  // and 0.01 radians in orientation
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);

  // We will create the request as a constraint using a helper function available
  // from the
  // `kinematic_constraints`_
  // package.
  //
  // .. _kinematic_constraints: http://docs.ros.org/indigo/api/moveit_core/html/namespacekinematic__constraints.html#a88becba14be9ced36fefc7980271e132
  req.group_name = "manipulator";
  moveit_msgs::Constraints pose_goal =
      kinematic_constraints::constructGoalConstraints("wrist_3_link", pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.push_back(pose_goal);

  // Now, call the pipeline and check whether planning was successful.
  planning_pipeline->generatePlan(planning_scene, req, res);
  /* Check that the planning was successful */
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }

  // Visualize the result
  // ^^^^^^^^^^^^^^^^^^^^
  ros::Publisher display_publisher =
      node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  /* Visualize the trajectory */
  ROS_INFO("Visualizing the trajectory");
  moveit_msgs::MotionPlanResponse response;
  res.getMessage(response);

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  display_publisher.publish(display_trajectory);

  sleep_time.sleep();

  ROS_INFO("Done");
  return 0;
}