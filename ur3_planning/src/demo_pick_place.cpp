
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

    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface plan_scene;
    std::map<std::string,geometry_msgs::Pose> mymap;
    std::vector<std::string> obj;
    obj.push_back("box1");  
    mymap = plan_scene.getObjectPoses(obj);
    auto search = mymap.find("box1");
    if(search != mymap.end()) {
        target_pose.pose.position.x = search->second.position.x - 0.1;
	    target_pose.pose.position.y = search->second.position.y;
	    target_pose.pose.position.z = search->second.position.z;
        std::cout << search->second.position.z << endl;
    }
    else {
        target_pose.pose.position.x = 0.4;
	    target_pose.pose.position.y = 0.4;
	    target_pose.pose.position.z = 0.2;
    }
    //ROS_ERROR("%s-%f %s-%f %s-%f",argv[1],atof(argv[1]),argv[2],atof(argv[2]),argv[3],atof(argv[3]));
    target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI,0.0,0.0); //horizontal
    //target_pose.pose.orientation.x = 0.5;
    
    //moveit_msgs::Constraints constraint;
    //constraint 

    group.setPoseTarget(target_pose);
    //group.setPathConstraints(constraint);
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
    else{
    	ROS_ERROR("Not found path!");
    	return 1;
    } 
    sleep(5);

    move_group.pick("box1");
    move_group.attachObject("box1");
    sleep(3);

    target_pose.pose.position.x = 0.6;
    target_pose.pose.position.y = 0.1;
    target_pose.pose.position.z = 0.1;
    group.setPoseTarget(target_pose);
    success = group.plan(my_plan);
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
    else{
    	ROS_ERROR("Not found path!");
    	return 1;
    } 
    sleep(5);
    geometry_msgs::PoseStamped place_pose;
    place_pose.pose.position.x = 0.4;
    place_pose.pose.position.y = 0.4;
    place_pose.pose.position.z = 0.2;
    move_group.place("box1",place_pose);
    move_group.detachObject("box1");
    sleep(5);

    group.setPositionTarget(0.6,0.1,0.1);
    success = group.plan(my_plan);
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
    else{
    	ROS_ERROR("Not found path!");
    	return 1;
    } 
    sleep(5);
    return 0;
}