#include <ros/ros.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>


int main(int argc, char **argv) {
    ros::init(argc, argv,"moveit_group_item");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nodeHandle;

    ros::Publisher planning_scene_diff_publisher = nodeHandle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  while (planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
    ros::WallDuration sleep_t(0.5);
    //sleep_t.sleep();
  }

  // Define the attached object message
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // We will use this message to add or
  // subtract the object from the world
  // and to attach the object to the robot
  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = "base_link";
  /* The header must contain a valid TF frame*/
  attached_object.object.header.frame_id = "base_link";
  /* The id of the object */
  char temp[10];
  sprintf(temp,"box%s",argv[1]);
  attached_object.object.id = temp;
  /* A default pose */
  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = atof(argv[2]);
  pose.position.y = atof(argv[3]);
  pose.position.z = atof(argv[4]);


  /* Define a box to be attached */
  float d1 = 0.2;
  float d2 = 0.2;
  float d3 = 0.1;
  if(argc > 5){
  	d1 = atof(argv[5]);
  	d2 = atof(argv[6]);
  	d3 = atof(argv[7]);
  }
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = d1;
  primitive.dimensions[1] = d2;
  primitive.dimensions[2] = d3;

  attached_object.object.primitives.push_back(primitive);
  attached_object.object.primitive_poses.push_back(pose);

  // Note that attaching an object to the robot requires
  // the corresponding operation to be specified as an ADD operation
  attached_object.object.operation = attached_object.object.ADD;

  // Add an object into the environment
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Add the object into the environment by adding it to
  // the set of collision objects in the "world" part of the
  // planning scene. Note that we are using only the "object"
  // field of the attached_object message here.
  ROS_INFO("Adding the object into the world at the location of the right wrist.");
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(attached_object.object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);
  usleep(1000);
    
  return 0;
}