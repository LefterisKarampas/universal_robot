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


using namespace std;

int main(int argc, char **argv) {
    //cout << argv[0] << endl;
    if(argc != 4){
        ROS_ERROR("FAILED waiting 3 positions for args");
        exit(1);
    }
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

    group.setStartStateToCurrentState();

    ROS_INFO("End effector reference frame: %s", group.getEndEffectorLink().c_str());
    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id="world";
    target_pose.header.stamp=ros::Time::now()+ros::Duration(10.0);
    //ROS_ERROR("%s-%f %s-%f %s-%f",argv[1],atof(argv[1]),argv[2],atof(argv[2]),argv[3],atof(argv[3]));
    target_pose.pose.position.x = atof(argv[1]);
    target_pose.pose.position.y = atof(argv[2]);
    target_pose.pose.position.z = atof(argv[3]);
    target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI,0.0,0.0); //horizontal
    //target_pose.pose.orientation.x = 0.5;
    
    moveit_msgs::Constraints constraint;
    //constraint 

    group.setPoseTarget(target_pose);
    group.setPathConstraints(constraint);
    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);
    ROS_INFO("plan: %s",success?"SUCCESS":"FAILED");
    if(success) {
        char q;
        std::cout << "Please make sure that your robot can move freely between these poses before proceeding!\n";
        std::cout << "Continue? y/n: ";
        std::cin >> q;
        std::cout << endl;
        if(q == 'y'){
           ROS_INFO("Moving...");
            //group.stop();
            group.execute(my_plan);
        }
    } 
    sleep(5);
    return 0;
}