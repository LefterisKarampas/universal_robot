#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/kinematic_constraints/utils.h>
//#include <eigen_conversions/eigen_msg.h>

bool userCallback(const robot_state::RobotState& kinematic_state, bool verbose)
{
  const double* joint_values = kinematic_state.getJointPositions("wrist_3_joint");
  return (joint_values[0] > 0.0);
}

int main(int argc,char **argv){
	ros::init(argc, argv, "right_arm_kinematics");
  	ros::AsyncSpinner spinner(1);
	spinner.start();

	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	planning_scene::PlanningScene planning_scene(kinematic_model);

	// Collision Checking
	collision_detection::CollisionRequest collision_request;
	collision_detection::CollisionResult collision_result;
	planning_scene.checkSelfCollision(collision_request, collision_result);
	ROS_INFO_STREAM("Test 1: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

	robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();
	current_state.setToRandomPositions();
	collision_result.clear();
	planning_scene.checkSelfCollision(collision_request, collision_result);
	ROS_INFO_STREAM("Test 2: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

	collision_request.group_name = "manipulator";
	current_state.setToRandomPositions();
	collision_result.clear();
	planning_scene.checkSelfCollision(collision_request, collision_result);
	ROS_INFO_STREAM("Test 3: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

	return 0;
}
