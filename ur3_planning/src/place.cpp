#include <ros/ros.h>
// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

int main(int argc,char **argv){
    if(argc != 5){
        ROS_ERROR("FAILED waiting 1 object & 3 positions for args");
        exit(1);
    }
    ros::init(argc, argv, "move_group_tutorial");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");

    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    geometry_msgs::PoseStamped place_pose;
    place_pose.pose.position.x = atof(argv[1])*0.1;
    place_pose.pose.position.y = atof(argv[2])*0.1;
    place_pose.pose.position.z = atof(argv[3])*0.1;
    move_group.place(argv[1],place_pose);
    move_group.detachObject(argv[1]);
    sleep(5);
    //move_group.place("box1");
    //move_group.detachObject("box1");
    std::cout << "EXIT!" <<std::endl;
    return 0;

}