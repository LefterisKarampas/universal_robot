#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <moveit_msgs/ExecuteKnownTrajectory.h>

int main(int argc, char **argv)
{
  // local variables
  float pi = 3.14159;

  ros::init(argc, argv, "move_cartesian_path");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Setup
  moveit::planning_interface::MoveGroup group("manipulator"); // note the name of the planning group. This is for UR# robots
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
 
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("End effector frame: %s", group.getEndEffectorLink().c_str());

  group.setPoseReferenceFrame("ee_link"); // all coordinates are relative to the position of the end effector
  moveit::planning_interface::MoveGroup::Plan my_plan;

 // Planning a Cartesian Path
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  group.startStateMonitor();
  group.setStartStateToCurrentState();
  group.setPlanningTime(12);

  ROS_INFO_STREAM("current EE pose: " << group.getCurrentPose());

    // Cartesian Paths
  // ^^^^^^^^^^^^^^^
  // You can plan a cartesian path directly by specifying a list of waypoints
  // for the end-effector to go through.
  std::vector<geometry_msgs::Pose> waypoints;

  // not sure about this, I think it worked, but I've solved it now by looking up some positions
  geometry_msgs::Pose ROBOT_START_POSE;
  ROBOT_START_POSE.position.x = 0; ROBOT_START_POSE.position.y = 0; ROBOT_START_POSE.position.z = 0;
  ROBOT_START_POSE.orientation.x = 0; ROBOT_START_POSE.orientation.y = 0;
  ROBOT_START_POSE.orientation.z = 0; ROBOT_START_POSE.orientation.w = 0;

  geometry_msgs::Pose cartesian_target = ROBOT_START_POSE;
  waypoints.push_back(cartesian_target); // Start by doing nothing (probably not necessary)
  cartesian_target.position.y = 0.2;
  waypoints.push_back(cartesian_target);

  cartesian_target.position.z = 0.2;
  cartesian_target.position.y = 0.0;
  waypoints.push_back(cartesian_target);

  cartesian_target.position.z = -0.2;
  cartesian_target.position.y = 0.0;
  waypoints.push_back(cartesian_target);

  cartesian_target.position.z = 0.0;
  cartesian_target.position.y = -0.2;
  waypoints.push_back(cartesian_target);
 
  // We want the cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in cartesian
  // translation. We will specify the jump threshold as 0.0, effectively
  // disabling it.
  // moveit_msgs::RobotTrajectory trajectory;
  moveit_msgs::ExecuteKnownTrajectory srv;
  double fraction = group.computeCartesianPath(waypoints,
                                               0.01, // eef_step
                                               0.0, // jump_threshold
                                               srv.request.trajectory, true);

  // The trajectory needs to be modified so it will include velocities as well.
  // First to create a RobotTrajectory object
  robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "manipulator");

  // Second get a RobotTrajectory from trajectory
  rt.setRobotTrajectoryMsg(*group.getCurrentState(), srv.request.trajectory);
 
  // Thrid create a IterativeParabolicTimeParameterization object
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  // Fourth compute computeTimeStamps
  bool success = iptp.computeTimeStamps(rt);
  ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
  // Get RobotTrajectory_msg from RobotTrajectory
  rt.getRobotTrajectoryMsg(srv.request.trajectory);

  ROS_INFO("Visualizing cartesian path plan (%.2f%% acheived)",
        fraction * 100.0);

    ros::ServiceClient client = node_handle.serviceClient<moveit_msgs::ExecuteKnownTrajectory>("/execute_kinematic_path");

    // send trajectory to arm controller
    srv.request.wait_for_execution = true;
    if (!client.call(srv))
    {
      ROS_INFO("Kinematic service call failed!");
    }

    if (srv.response.error_code.val < 1)
    {
      ROS_INFO_STREAM("Kinematic path service call returned error code : " << srv.response.error_code.val);
    } else
    {
      ROS_INFO("Sent Kinematic path to server");
    }

  ros::shutdown();
  return 0;
}