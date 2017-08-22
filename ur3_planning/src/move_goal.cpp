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

/*bool userCallback(const robot_state::RobotState& kinematic_state, bool verbose)
{
  const double* joint_values = kinematic_state.getJointPositions("wrist_3_joint");
  return (joint_values[0] > 0.0);
}*/

int main(int argc, char **argv) {
    //cout << argv[0] << endl;
    if(argc <= 3){
        ROS_ERROR("FAILED waiting 3+ positions for args");
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
    //group.allowReplanning(true);
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setPoseReferenceFrame("world");
	group.setGoalTolerance(0.001);

    group.setStartStateToCurrentState();

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    //robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    //planning_scene::PlanningScene planning_scene(kinematic_model);
    //planning_scene.setStateFeasibilityPredicate(userCallback);

    robot_state::RobotState start_state(*group.getCurrentState());
    robot_model::RobotModelPtr robot_model_ptr = robot_model_loader.getModel();
    const robot_state::JointModelGroup* joint_model_group = robot_model_ptr->getJointModelGroup("manipulator");

    ROS_INFO("End effector reference frame: %s", group.getEndEffectorLink().c_str());
    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id="world";
    target_pose.header.stamp=ros::Time::now()+ros::Duration(10.0);
    //ROS_ERROR("%s-%f %s-%f %s-%f",argv[1],atof(argv[1]),argv[2],atof(argv[2]),argv[3],atof(argv[3]));
    target_pose.pose.position.x = atof(argv[1]);
    target_pose.pose.position.y = atof(argv[2]);
    target_pose.pose.position.z = atof(argv[3]);

    //target_pose.pose.orientation.w = 1.0;
    geometry_msgs::Pose target_pose1;
    target_pose1.position.x = target_pose.pose.position.x;
    target_pose1.position.y = target_pose.pose.position.y;
    target_pose1.position.z = target_pose.pose.position.z;
    if(argc > 4){
        target_pose1.orientation = tf::createQuaternionMsgFromRollPitchYaw(atof(argv[4]),atof(argv[5]),atof(argv[6]));
        target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(atof(argv[4]),atof(argv[5]),atof(argv[6])); //horizontal
    }
    else{
        target_pose1.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI,0.0,0.0);
        target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI,0.0,0.0); //horizontal
    }
    bool found_ik = start_state.setFromIK(joint_model_group, target_pose1);
    if(!found_ik){
        ROS_ERROR("Invalid end point!");
        return 2;
    }
    moveit_msgs::Constraints constraint;
    //constraint 
    
    //bool state_feasible = planning_scene.isStateFeasible(start_state);
    //ROS_INFO_STREAM("State is " << (state_feasible ? "feasible" : "not feasible"));

    //bool state_valid = planning_scene.isStateValid(start_state, kinematic_constraint_set, "manipulator");
    //ROS_INFO_STREAM("Test 10: Random state is " << (state_valid ? "valid" : "not valid"));

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
        std::cout << std::endl;
        if(q == 'y'){
           ROS_INFO("Moving...");
            //group.stop();
            group.asyncExecute(my_plan);
        }
    } 
    sleep(5);
    return 0;
}