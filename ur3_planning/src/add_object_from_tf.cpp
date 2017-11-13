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
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <eigen_conversions/eigen_msg.h>
#include <camera_test/coke.h>

using namespace std;


void add_object(string name,double x,double y,double z,double rx,double ry,double rz,double rw){
    moveit::planning_interface::PlanningSceneInterface current_scene;
    sleep(2.0);
    moveit_msgs::CollisionObject co;
    shapes::Mesh* m = shapes::createMeshFromResource("file:///home/lkarampas/coke.stl");
    shape_msgs::Mesh co_mesh;
    shapes::ShapeMsg co_mesh_msg;
    shapes::constructMsgFromShape(m,co_mesh_msg);
    co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);

    co.meshes.resize(1);
    co.mesh_poses.resize(1);
    co.meshes[0] = co_mesh;
    co.header.frame_id = "world";
    co.id = name;  
    co.mesh_poses[0].position.x = x;
    co.mesh_poses[0].position.y = y;
    co.mesh_poses[0].position.z = z;
    co.mesh_poses[0].orientation.w= rw; 
    co.mesh_poses[0].orientation.x= ry; 
    co.mesh_poses[0].orientation.y= rz;
    co.mesh_poses[0].orientation.z= rz;   

    co.meshes.push_back(co_mesh);
    co.mesh_poses.push_back(co.mesh_poses[0]);
    co.operation = co.ADD;
    std::vector<moveit_msgs::CollisionObject> vec;
    vec.push_back(co);
    ROS_INFO("%s added into the world",name.c_str());
    current_scene.addCollisionObjects(vec);
    sleep(5.0);
}


void add_objects(const camera_test::coke & frame){
    /*Set planning parameters*/
    moveit::planning_interface::MoveGroup group("manipulator"); 
    group.setStartStateToCurrentState();
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_state::RobotState start_state(*group.getCurrentState());                                      //Set start state for the planning
    robot_model::RobotModelPtr robot_model_ptr = robot_model_loader.getModel();
    const robot_state::JointModelGroup* joint_model_group = robot_model_ptr->getJointModelGroup("manipulator");
    
    
    tf::TransformListener listener;
    tf::StampedTransform transform;
    for(int i =0;i<frame.size;i++){
        try{
            listener.waitForTransform("/world", frame.tf_names[i],ros::Time(0),ros::Duration(3.0));
            listener.lookupTransform("/world", frame.tf_names[i],  
                                   ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        geometry_msgs::PoseStamped target_pose;
        target_pose.header.frame_id="world";
        target_pose.header.stamp=ros::Time::now()+ros::Duration(10.0);

        /*Set planning position */
        target_pose.pose.position.x = transform.getOrigin().x();
        target_pose.pose.position.y = transform.getOrigin().y();
        target_pose.pose.position.z = transform.getOrigin().z();

        /*Set position for checking validity for goal state with IK Solver*/
        geometry_msgs::Pose target_pose1;
        target_pose1.position = target_pose.pose.position;
        
        //Set the orientation
       
        target_pose1.orientation.x = transform.getRotation().x();
        target_pose1.orientation.x = transform.getRotation().y();
        target_pose1.orientation.x = transform.getRotation().z();
        target_pose1.orientation.x = transform.getRotation().w();
        target_pose.pose.orientation = target_pose1.orientation;

        
        //Check if the goal state is valid 
        bool found_ik = start_state.setFromIK(joint_model_group, target_pose1);
        if(!found_ik){
            ROS_ERROR("Invalid end point!");
            continue;
        }
        add_object(frame.tf_names[i],target_pose1.position.x,target_pose1.position.y,target_pose1.position.z,target_pose1.orientation.x,target_pose1.orientation.y,target_pose1.orientation.z,target_pose1.orientation.w);
        sleep(2);
    }
    exit(1);
}


int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "create_object_from_tf");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe ("coke_tf", 1, add_objects);
    // Spin
    ros::spin ();
} 