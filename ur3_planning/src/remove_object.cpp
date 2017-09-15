
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv,"moveit_group_remove");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nodeHandle;

    ros::Publisher planning_scene_diff_publisher = nodeHandle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
	while (planning_scene_diff_publisher.getNumSubscribers() < 1)
	{
		ros::WallDuration sleep_t(0.5);
	    //sleep_t.sleep();
	}

	moveit_msgs::CollisionObject remove_object;
	int i;
	for(i=1;i<argc;i++){
		remove_object.id = argv[i];
		remove_object.header.frame_id = "base_link";
		remove_object.operation = remove_object.REMOVE;

		ROS_INFO("Removing the object from the world.");
		moveit_msgs::PlanningScene planning_scene;
		planning_scene.is_diff = true;
	  	planning_scene_diff_publisher.publish(planning_scene);
		planning_scene.robot_state.attached_collision_objects.clear();
		planning_scene.world.collision_objects.clear();
		planning_scene.world.collision_objects.push_back(remove_object);
		planning_scene_diff_publisher.publish(planning_scene);
		sleep(2);
	}
	return 0;

}