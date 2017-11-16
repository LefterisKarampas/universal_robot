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
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
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
string object_name = "grasp_object";

int Cartesian_Path(moveit::planning_interface::MoveGroup & group,std::vector<geometry_msgs::Pose> waypoints){
    moveit_msgs::ExecuteKnownTrajectory srv;
    double fraction = group.computeCartesianPath(waypoints,
                                                                                   0.001, // eef_step
                                                                                   0.0, // jump_threshold
                                                                                   srv.request.trajectory, true);

    robot_trajectory::RobotTrajectory rt (group.getCurrentState()->getRobotModel(), "manipulator");
    rt.setRobotTrajectoryMsg(*group.getCurrentState(), srv.request.trajectory); 

    // create a IterativeParabolicTimeParameterization object
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    bool success = iptp.computeTimeStamps(rt);
    ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

    // Get RobotTrajectory_msg from RobotTrajectory
    rt.getRobotTrajectoryMsg(srv.request.trajectory);
    // Finally plan and execute the trajectory
    moveit::planning_interface::MoveGroup::Plan plan;
    plan.trajectory_ = srv.request.trajectory;
    ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)",fraction * 100.0);

    if (fraction >= 0.85 && success == true) {
        group.asyncExecute(plan);
        sleep(2);
        return 1;
    }
    else{
        ROS_ERROR("FAIL planning!");
        return 0;
    }

}

void remove_object(string name){
    ros::NodeHandle nodeHandle;
    ros::Publisher planning_scene_diff_publisher = nodeHandle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        ros::WallDuration sleep_t(0.5);
        //sleep_t.sleep();
    }

    moveit_msgs::CollisionObject remove_object;
   
    remove_object.id = name;
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

void add_object(string name,geometry_msgs::Pose pose){
    ros::NodeHandle node_handle;
    ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
      ros::WallDuration sleep_t(0.5);
    }
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
    pose.position.z = 0.86;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;
    co.mesh_poses[0] = pose;
    co.meshes.push_back(co_mesh);
    co.mesh_poses.push_back(co.mesh_poses[0]);
    co.operation = co.ADD;
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(co);
    planning_scene.is_diff = true;
    ROS_INFO("%s added into the world",name.c_str());
    for(int i=0;i<5;i++)
      planning_scene_diff_publisher.publish(planning_scene);
}



int open_gripper(){
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<schunk_pg70::set_position>("schunk_pg70/set_position");
    schunk_pg70::set_position srv;
    double pos = 68;
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


int main(int argc,char * argv[]){
    ros::init(argc, argv,"move_grasp");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nodeHandle;
    /*Set planning parameters*/
    moveit::planning_interface::MoveGroup group("manipulator"); 
    group.setMaxVelocityScalingFactor(0.5); 
    group.setMaxAccelerationScalingFactor(0.5);
    group.setPlanningTime(5.0);
    group.setNumPlanningAttempts(100);
    group.allowReplanning(true);
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setPoseReferenceFrame("world");
  group.setGoalTolerance(0.001);

    group.setStartStateToCurrentState();

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

    robot_state::RobotState start_state(*group.getCurrentState());                    //Set start state for the planning
    robot_model::RobotModelPtr robot_model_ptr = robot_model_loader.getModel();
    const robot_state::JointModelGroup* joint_model_group = robot_model_ptr->getJointModelGroup("manipulator");


    bool goal_reached = false;
    tf::TransformListener listener;
    open_gripper();
    while(ros::ok()&& !goal_reached){
        tf::StampedTransform transform;
        try{
            listener.waitForTransform("world","grasp",ros::Time(0),ros::Duration(3.0));
            listener.lookupTransform("/world", "grasp",  
                                   ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        geometry_msgs::PoseStamped target_pose;
        target_pose.header.frame_id="world";
        target_pose.header.stamp=ros::Time::now()+ros::Duration(10.0);

        /*Set planning position */
        target_pose.pose.position.x = transform.getOrigin().x();
        target_pose.pose.position.y = transform.getOrigin().y();
        target_pose.pose.position.z = 0.93;
        /*Set position for checking validity for goal state with IK Solver*/
        geometry_msgs::Pose target_pose1;
        target_pose1.position = target_pose.pose.position;
        
        //Set the orientation
        target_pose1.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,M_PI/2,0);
        /*target_pose1.orientation.x = 0;
        target_pose1.orientation.y = 0;
        target_pose1.orientation.z = 0;
        target_pose1.orientation.w = 1;*/
        target_pose.pose.orientation = target_pose1.orientation;

        //open_gripper(); 
        //Check if the goal state is valid
        bool found_ik = start_state.setFromIK(joint_model_group, target_pose1);
        if(!found_ik){
            ROS_ERROR("Invalid end point!");
            continue;
        }
        add_object(object_name,target_pose1);
        sleep(5);
        target_pose.pose.position.x -= 0.10;
        target_pose.pose.position.y += 0.025;
        target_pose.pose.position.z = 1.1;
        group.setPoseTarget(target_pose);     //Set goal pose

        moveit::planning_interface::MoveGroup::Plan my_plan;  //Create plan object
        bool success = group.plan(my_plan);           //if success == true -> my_plan contains the planning from start state to goal state
        ROS_INFO("plan: %s",success?"SUCCESS":"FAILED");
        if(success) {
            char q;
            std::cout << "Please make sure that your robot can move freely between these poses before proceeding!\n";
            std::cout << "Continue? y/n: ";
            std::cin >> q;
            std::cout << std::endl;
            if(q == 'y'){
               ROS_INFO("Moving...");
                group.asyncExecute(my_plan);
                goal_reached = true;
            }
            else{
                continue;
            }
        }
        else{
            continue;
        }
        sleep(10);
        //emove_object(object_name);
        //sleep(1);
        std::vector<geometry_msgs::Pose> waypoints;
        target_pose1.position = target_pose.pose.position;
        remove_object(object_name);
        sleep(1);
        waypoints.push_back(target_pose1);
        target_pose1.position.x += 0.155;
        waypoints.push_back(target_pose1);
        while(!Cartesian_Path(group,waypoints));
        sleep(5);
        waypoints.clear();
       // target_pose1.position.z += 0.15;
        //waypoints.push_back(target_pose1);
        close_gripper();
        sleep(2);
        //group.attachObject(object_name);
        sleep(1);
        Cartesian_Path(group,waypoints);
        sleep(2);

        target_pose.pose.position.x = 0.2627;
        target_pose.pose.position.y = 0.2661;
        target_pose.pose.position.z = 1.0945;
        target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,M_PI/2,0);
        group.setPoseTarget(target_pose);                   //Set goal pose
        success = false;
        bool g_r = false;
        while(ros::ok() && !g_r){
            success = group.plan(my_plan);                     //if success == true -> my_plan contains the planning from start state to goal state
            ROS_INFO("plan: %s",success?"SUCCESS":"FAILED");
            if(success) {
                char q;
                std::cout << "Please make sure that your robot can move freely between these poses before proceeding!\n";
                std::cout << "Continue? y/n: ";
                std::cin >> q;
                std::cout << std::endl;
                if(q == 'y'){
                   ROS_INFO("Moving...");
                    group.asyncExecute(my_plan);
                    g_r = true;
                }
                else{
                    continue;
                }
            }
            else{
                continue;
            }
        }
        sleep(10);
        open_gripper();
        sleep(1);
        //group.detachObject(object_name);
    }
}