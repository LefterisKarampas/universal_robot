#include <ros/ros.h>
#include <iostream>
#include <string>
#include <stdlib.h>     /* atof */
#include <unistd.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/Constraints.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

int main(int argc,char *argv[]){
	if(argc != 4){
        ROS_ERROR("FAILED waiting 3 positions for args");
        exit(1);
    }
    ros::init(argc, argv,"moveit_group_goal");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nodeHandle;
	geometry_msgs::Pose targetPose;
	targetPose.position.x = atof(argv[1]);
	targetPose.position.y = atof(argv[2]);
	targetPose.position.z = atof(argv[3]);
	targetPose.orientation.w = 1.0;
	//joint space goal

	moveit::planning_interface::MoveGroup group("manipulator");
	group.setMaxVelocityScalingFactor(0.1);
    group.setMaxAccelerationScalingFactor(0.1);
    group.setPlanningTime(10.0);
    group.setNumPlanningAttempts(100);
    //group.setPoseReferenceFrame("world");
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr robot_model_ptr = robot_model_loader.getModel();
	robot_state::RobotStatePtr robot_state_ptr;
    robot_state_ptr = group.getCurrentState();
    group.setStartState(*robot_state_ptr);
	//robot_state_ptr->setToDefaultValues();
	//group.setStartStateToCurrentState();
	const robot_state::JointModelGroup* joint_model_group = robot_model_ptr->getJointModelGroup(group.getName());

	bool found_ik = robot_state_ptr->setFromIK(joint_model_group, targetPose, 10,1);

	std::vector<double> jointValues;
	robot_state_ptr->copyJointGroupPositions("manipulator", jointValues);

	ROS_INFO("joint_values: %f %f %f %f %f %f",jointValues[0],jointValues[1],jointValues[2],jointValues[3],jointValues[4],jointValues[5]);
	group.setJointValueTarget(jointValues);
	//group.setMaxVelScalingFactor(0.1);

	moveit::planning_interface::MoveGroup::Plan plan;
	bool success = group.plan(plan);
	ROS_INFO("Visualizing plan  (pose goal) %s",success?"":"FAILED");
	if(success) {
        char q;
        std::cout << "Please make sure that your robot can move freely between these poses before proceeding!\n";
        std::cout << "Continue? y/n: ";
        std::cin >> q;
        std::cout << std::endl;
        if(q == 'y'){
           ROS_INFO("Moving...");
            //group.stop();
            group.execute(plan);
        }
    } 
    sleep(5);
	//currentPose = group.getCurrentPose("ee_link");
	//    ROS_INFO("Final pose_x: %f\n current_pose_y :%f\n current_pose_z: %f",currentPose.pose.position.x,currentPose.pose.position.y, currentPose.pose.position.z);
	if (success)
		return 0;
	else
		return 1;
}