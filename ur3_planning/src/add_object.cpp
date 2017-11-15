#include <ros/ros.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

using namespace std;

int main(int argc, char **argv) {
    if (argc < 5){
        return 0;
    }
    ros::init(argc, argv,"moveit_group_item");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nodeHandle;

    ros::Publisher planning_scene_diff_publisher = nodeHandle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
      ros::WallDuration sleep_t(0.5);
    }

  
  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = "base_link";
  attached_object.object.header.frame_id = "base_link";
  char *temp;
  temp = (char *)malloc(sizeof(argv[1])+1);
  sprintf(temp,"%s",argv[1]);
  temp[sizeof(argv[1])] = '\0';
  attached_object.object.id = temp;

  /* Set pose */
  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = atof(argv[2])*0.01;
  pose.position.y = atof(argv[3])*0.01;
  pose.position.z = atof(argv[4])*0.01;

  /* Define box size */
  float d1 = 0.2;
  float d2 = 0.2;
  float d3 = 0.1;
  if(argc > 5){
  	d1 = atof(argv[5]);
  	d2 = atof(argv[6]);
  	d3 = atof(argv[7]);
  }
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;       //We create a box if you want something else just change this line
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = d1;
  primitive.dimensions[1] = d2;
  primitive.dimensions[2] = d3;

  attached_object.object.primitives.push_back(primitive);
  attached_object.object.primitive_poses.push_back(pose);

  // Note that attaching an object to the robot requires
  // the corresponding operation to be specified as an ADD operation
  attached_object.object.operation = attached_object.object.ADD;      //Type: Add new object

  moveit_msgs::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(attached_object.object); //Add the new object in the planning scene
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);                    //Publish the new scene
  usleep(1000);
  free(temp);
  return 0;
}
