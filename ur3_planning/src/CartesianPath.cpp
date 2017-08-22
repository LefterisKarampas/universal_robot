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
#include <moveit_msgs/ExecuteKnownTrajectory.h>


using namespace std;

int main(int argc, char **argv) {
    //cout << argv[0] << endl;
    ros::init(argc, argv,"moveit_group_goal");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nodeHandle;

    moveit::planning_interface::MoveGroup group("manipulator");
    moveit_msgs::ExecuteKnownTrajectory srv;
    moveit::planning_interface::MoveGroup::Plan plan;
    group.setMaxVelocityScalingFactor(0.1);
    group.setMaxAccelerationScalingFactor(0.1);
    group.setPlanningTime(30.0);
    group.setNumPlanningAttempts(100);
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setPoseReferenceFrame("world");
    group.setGoalTolerance(0.01);


  
	std::vector<geometry_msgs::Pose> waypoints;
	geometry_msgs::Pose target_pose;
	for(int i=1;i<argc;i+=3){
		geometry_msgs::Pose target_pose;
		target_pose.position.x = atof(argv[i]);
		target_pose.position.y = atof(argv[i+1]);
		target_pose.position.z = atof(argv[i+2]);
		waypoints.push_back(target_pose);
	}
	//waypoints.push_back(target_pose2);
  	double fraction = group.computeCartesianPath(waypoints,
                                                                                   0.001, // eef_step
                                                                                   0.0, // jump_threshold
                                                                                   srv.request.trajectory, true);

	robot_trajectory::RobotTrajectory rt (group.getCurrentState()->getRobotModel(), "manipulator");
	rt.setRobotTrajectoryMsg(*group.getCurrentState(), srv.request.trajectory);

	// create a IterativeParabolicTimeParameterization object
	trajectory_processing::IterativeParabolicTimeParameterization iptp;
	bool success = iptp.computeTimeStamps(rt);
	ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

	// Get RobotTrajectory_msg from RobotTrajectory
	rt.getRobotTrajectoryMsg(srv.request.trajectory);
	// Finally plan and execute the trajectory
	plan.trajectory_ = srv.request.trajectory;
	ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)",fraction * 100.0);

	if (fraction >= 0.65 && success == true) {
		char q;
		std::cout << "Please make sure that your robot can move freely between these poses before proceeding!\n";
        std::cout << "Continue? y/n: ";
        std::cin >> q;
        std::cout << std::endl;
        if(q == 'y'){
           ROS_INFO("Moving...");
            group.execute(plan);
        }
	} else{
		ROS_ERROR("FAIL planning!");
	}

  return 0;
}