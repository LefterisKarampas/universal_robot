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
#include <schunk_pg70/set_position.h>
#include <cstdlib>
#include <camera_test/coke.h> 

using namespace std;

static int k = 1;
static string var;

void add_object(double x,double y,double z,double rx,double ry,double rz,double rw){
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
    co.header.frame_id = "wall"; 
    co.id = var;  
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
    ROS_INFO("Wall added into the world");
    current_scene.addCollisionObjects(vec);
    sleep(5.0);
}

int open_gripper(){
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<schunk_pg70::set_position>("schunk_pg70/set_position");
    schunk_pg70::set_position srv;
    double pos = 60;
    srv.request.goal_position = pos;
    double vel = 60;
    srv.request.goal_velocity = vel;
    srv.request.goal_acceleration = 250;  
    if (client.call(srv))
    {
        ROS_INFO("schunk_pg70 is moving to goal position!");
    }
    else
    {
        ROS_ERROR("Failed to call service for pg70 moving");
        return 1;
    }
    sleep(3);
    return 0;
}

int close_gripper(){
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<schunk_pg70::set_position>("schunk_pg70/set_position");
    schunk_pg70::set_position srv;
    double pos = 10;
    srv.request.goal_position = pos;
    double vel = 60;
    srv.request.goal_velocity = vel;
    srv.request.goal_acceleration = 250;  
    if (client.call(srv))
    {
        ROS_INFO("schunk_pg70 is moving to goal position!");
    }
    else
    {
        ROS_ERROR("Failed to call service for pg70 moving");
        return 1;
    }
    sleep(3);
    return 0;
}


void object_frames(const camera_test::coke & frame){
    /*Set planning parameters*/
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

    robot_state::RobotState start_state(*group.getCurrentState());										//Set start state for the planning
    robot_model::RobotModelPtr robot_model_ptr = robot_model_loader.getModel();
    const robot_state::JointModelGroup* joint_model_group = robot_model_ptr->getJointModelGroup("manipulator");


    bool goal_reached = false;
    tf::TransformListener listener;
    for(int i =0;i<2;i++){
        while(ros::ok()&& !goal_reached){
            tf::StampedTransform transform;
            std::ostringstream oss;
            if(i == 0)
                oss << "red" << k;
            else
                oss << "brown" << k;
            var = oss.str();
            try{
                listener.lookupTransform("/world", var,  
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

            while(open_gripper()); 
            //Check if the goal state is valid
            bool found_ik = start_state.setFromIK(joint_model_group, target_pose1);
            if(!found_ik){
                ROS_ERROR("Invalid end point!");
                continue;
            }
            add_object(target_pose1.position.x,target_pose1.position.y,target_pose1.position.z,target_pose1.orientation.x,target_pose1.orientation.y,target_pose1.orientation.z,target_pose1.orientation.w);
            //while(close_gripper());
            k++;
            //target_pose.pose.position.x -= 0.2;
            //target_pose.pose.position.y -= 0.2;
            /*target_pose.pose.position.z += 0.4;
            group.setPoseTarget(target_pose);			//Set goal pose

            moveit::planning_interface::MoveGroup::Plan my_plan;	//Create plan object
            bool success = group.plan(my_plan);						//if success == true -> my_plan contains the planning from start state to goal state
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
                    goal_reached = true;
                }
            }*/
            goal_reached = true;
        } 
    }
    sleep(5);
}

int main(int argc,char * argv[]){
    ros::init(argc, argv,"moveit_to_tf");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe ("colors", 1, object_frames);
    ros::spin();
}