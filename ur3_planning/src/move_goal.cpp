#include <ros/ros.h>
#include <iostream>
#include <string>
#include <stdlib.h>     /* atof */
#include <unistd.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/Constraints.h>
#include <moveit/robot_model_loader/robot_model_loader.h>


using namespace std;
int main(int argc, char **argv) {
    if(argc <= 3){
        ROS_ERROR("FAILED waiting 3+ positions for args");
        exit(1);
    }
    ros::init(argc, argv,"moveit_group_goal");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nodeHandle;
    /*Set planning parameters*/
    moveit::planning_interface::MoveGroup group("manipulator"); 
    group.setMaxVelocityScalingFactor(0.1);	
    group.setMaxAccelerationScalingFactor(0.1);
    group.setPlanningTime(5.0);
    group.setNumPlanningAttempts(100);
    //group.allowReplanning(true);
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setPoseReferenceFrame("/world");
	group.setGoalTolerance(0.001);

    group.setStartStateToCurrentState();

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

    robot_state::RobotState start_state(*group.getCurrentState());										//Set start state for the planning
    robot_model::RobotModelPtr robot_model_ptr = robot_model_loader.getModel();
    const robot_state::JointModelGroup* joint_model_group = robot_model_ptr->getJointModelGroup("manipulator");

    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id="world";
    target_pose.header.stamp=ros::Time::now()+ros::Duration(10.0);

    /*Set planning position */
    target_pose.pose.position.x = atof(argv[1])*0.01;
    target_pose.pose.position.y = atof(argv[2])*0.01;
    target_pose.pose.position.z = atof(argv[3])*0.01;

    /*Set position for checking validity for goal state with IK Solver*/
    geometry_msgs::Pose target_pose1;
    target_pose1.position.x = target_pose.pose.position.x;
    target_pose1.position.y = target_pose.pose.position.y;
    target_pose1.position.z = target_pose.pose.position.z;

    /*Set the orientation */
    if(argc > 4){
        target_pose1.orientation = tf::createQuaternionMsgFromRollPitchYaw(atof(argv[4]),atof(argv[5]),atof(argv[6]));
        target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(atof(argv[4]),atof(argv[5]),atof(argv[6])); 
    }
    else{
        target_pose1.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI,0.0,0.0);
        target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI,0.0,0.0); //horizontal
    }

    /*Check if the goal state is valid */
    bool found_ik = start_state.setFromIK(joint_model_group, target_pose1);
    if(!found_ik){
        ROS_ERROR("Invalid end point!");
        return 2;
    }
    
    moveit_msgs::Constraints constraint;
    /*We can insert joint constraints in planning*/

    /*moveit_msgs::JointConstraint joint_constraints;
    joint_constraints.joint_name = "elbow_joint";
    joint_constraints.position = 1.2;
    joint_constraints.tolerance_above = 1.5;
    joint_constraints.tolerance_below = 1.5;
    joint_constraints.weight = 1.0;
    constraint.joint_constraints.push_back(joint_constraints);*/

    group.setPoseTarget(target_pose);			//Set goal pose
    group.setPathConstraints(constraint);		//Set constraints


    moveit::planning_interface::MoveGroup::Plan my_plan;	//Create plan object
    bool goal_reached = false;
    int loop = 0;
    while(ros::ok() && !goal_reached && loop < 5){
        bool success = group.plan(my_plan);						//if success == true -> my_plan contains the planning from start state to goal state
        ROS_INFO("plan: %s",success?"SUCCESS":"FAILED");
        if(success) {
            char q;
            std::cout << "Please make sure that your robot can move freely between these poses before proceeding!\n";
            std::cout << "Continue? y/n: ";
            std::cin >> q;
            std::cout << std::endl;
            if(q == 'y'){
               ROS_INFO("Moving...");
                //group.stop();
                group.asyncExecute(my_plan);
                goal_reached = true;
            }
        }
        loop++;
   } 
    sleep(5);
    return 0;
}