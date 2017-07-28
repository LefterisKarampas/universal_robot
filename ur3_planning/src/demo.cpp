#include <ros/ros.h>
#include <iostream>
#include <string>
#include <stdlib.h>     /* atof */
#include <unistd.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

using namespace std;

int main(int argc, char **argv) {
    //cout << argv[0] << endl;
    ros::init(argc, argv,"moveit_group_goal");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nodeHandle;

    moveit::planning_interface::MoveGroup group("manipulator");
    group.setMaxVelocityScalingFactor(1.0);
    group.setMaxAccelerationScalingFactor(1.0);
    group.setPlanningTime(5.0);
    group.setNumPlanningAttempts(100);
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setPoseReferenceFrame("world");
	group.setGoalTolerance(0.1);

    group.setStartStateToCurrentState();

    ROS_INFO("End effector reference frame: %s", group.getEndEffectorLink().c_str());
    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id="world";
    target_pose.header.stamp=ros::Time::now()+ros::Duration(10.0);
    //ROS_ERROR("%s-%f %s-%f %s-%f",argv[1],atof(argv[1]),argv[2],atof(argv[2]),argv[3],atof(argv[3]));
    target_pose.pose.position.x = -0.19725;
    target_pose.pose.position.y = -0.2765;
    target_pose.pose.position.z = 0.32584;
    //target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI,0.0,0.0); //horizontal
    target_pose.pose.orientation.x = 0.052269;
    target_pose.pose.orientation.y = 0.4292;
    target_pose.pose.orientation.z = -0.51939;
    target_pose.pose.orientation.w = 0.4292;
    

    group.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);
    ROS_INFO("plan: %s",success?"SUCCESS":"FAILED");
    if(success) {
        ROS_INFO("Moving...");
        group.execute(my_plan);
    } 
    sleep(3);
    target_pose.pose.position.x = 0.19725;
    target_pose.pose.position.y = -0.2765;
    target_pose.pose.position.z = 0.32584;
    //target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI,0.0,0.0); //horizontal
    group.setPoseTarget(target_pose);
    success = group.plan(my_plan);
    ROS_INFO("plan: %s",success?"SUCCESS":"FAILED");
    if(success) {
        ROS_INFO("Moving...");
        group.execute(my_plan);
    } 
    sleep(3);
    target_pose.pose.position.x = 0.0;
    target_pose.pose.position.y = 0.0;
    target_pose.pose.position.z = 0.65;
    group.setPoseTarget(target_pose);
    success = group.plan(my_plan);
    ROS_INFO("plan: %s",success?"SUCCESS":"FAILED");
    if(success) {
        ROS_INFO("Moving...");
        group.execute(my_plan);
    } 
    sleep(3);
    return 0;
}