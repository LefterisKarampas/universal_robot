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
    if(argc != 4){
        ROS_ERROR("FAILED waiting 3 positions for args");
        exit(1);
    }
    ros::init(argc, argv, "move_group_tutorial");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");

    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
   /* ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
    //moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    //ROS_INFO_NAMED("tutorial","Positions: %s",current_state->printStatePositions());
    std::vector<std::string> myvector = move_group.getJointNames();
    std::cout << "Active Joints Name:" << std::endl;
    for(int i=0;i<myvector.size();i++){
        std::cout << myvector[i] << std::endl;
    }
    std::vector<std::string> myvector1 = move_group.getJointNames();
    std::cout << "Links Name:" << std::endl;
    for(int i=0;i<myvector1.size();i++){
        std::cout << myvector1[i] << std::endl;
    }
    std::cout << "DOF: " << move_group.getVariableCount() << std::endl;
    move_group.setPositionTarget(0.0,0.0,0.65,"ee_link");
    move_group.setMaxVelocityScalingFactor(1.0);
    move_group.setMaxAccelerationScalingFactor(1.0);
    move_group.setPlanningTime(5.0);
    move_group.setNumPlanningAttempts(100);
    move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group.setPoseReferenceFrame("world");
    move_group.setGoalTolerance(0.1);
    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = move_group.plan(my_plan);
    ROS_INFO("plan: %s",success?"SUCCESS":"FAILED");
    if(success) {
        ROS_INFO("Moving...");
        move_group.execute(my_plan);
    } 
    sleep(5);*/
    //move_group.pick("box1");
    //move_group.attachObject("box1");
    geometry_msgs::PoseStamped place_pose;
    place_pose.pose.position.x = atof(argv[1]);
    place_pose.pose.position.y = atof(argv[2]);
    place_pose.pose.position.z = atof(argv[3]);
    move_group.place("box1",place_pose);
    move_group.detachObject("box1");
    sleep(5);
    move_group.place("box1");
    move_group.detachObject("box1");
   
    std::cout << "EXIT!" <<std::endl;
    return 0;

}