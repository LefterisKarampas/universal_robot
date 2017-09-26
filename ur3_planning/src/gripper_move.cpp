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
    if(argc != 2){
        ROS_ERROR("FAILED waiting 1 position arg");
        exit(1);
    }
    ros::init(argc, argv,"gripper_move");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nodeHandle;

    moveit::planning_interface::MoveGroup group("gripper");
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

    std::vector<double> jointValues;
    jointValues.push_back(atof(argv[1])*0.005);
    jointValues.push_back(atof(argv[1])*0.005);
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
    if (success)
        return 0;
    else
        return 1;
}