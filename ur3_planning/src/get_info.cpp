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

	ros::init(argc, argv, "move_group_tutorial");
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::NodeHandle node_handle("~");

	static const std::string PLANNING_GROUP = "manipulator";
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
	ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
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
    sleep(5);
    /*geometry_msgs::PoseStamped pos = move_group.getCurrentPose();
    std::cout << "X: " << pos.pose.position.x << std::endl;
    std::cout << "Y: " << pos.pose.position.y << std::endl;
    std::cout << "Z: " << pos.pose.position.z << std::endl;
    pos = move_group.getRandomPose("ee_link");
    move_group.setPoseTarget(pos,"ee_link");
    success = move_group.plan(my_plan);
    ROS_INFO("plan: %s",success?"SUCCESS":"FAILED");
    if(success) {
        ROS_INFO("Moving...");
        move_group.execute(my_plan);
    }
    sleep(5); */
    moveit_msgs::Constraints ocm;
    for(int i =0;i<ocm.joint_constraints.size();i++){
    	std::cout << ocm.joint_constraints[i].joint_name << std::endl;
    }
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::vector<std::string> object = planning_scene_interface.getKnownObjectNames();
    for(int i =0;i<object.size();i++){
    	std::cout << object[i] << std::endl;
    }
	std::cout << "EXIT!" <<std::endl;
	return 0;

}