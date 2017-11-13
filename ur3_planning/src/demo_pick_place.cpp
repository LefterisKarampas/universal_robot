
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <stdlib.h>     /* atof */
#include <unistd.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/Constraints.h>
#include <schunk_pg70/set_position.h>

using namespace std;

static int k = 1;
static std::string var;

void pick(moveit::planning_interface::MoveGroup &group,geometry_msgs::PoseStamped &p)
{
  std::vector<moveit_msgs::Grasp> grasps;

  p.header.frame_id = "ee_link";
  moveit_msgs::Grasp g;
  g.grasp_pose = p;

  g.pre_grasp_approach.direction.vector.x = 1.0;
  g.pre_grasp_approach.direction.header.frame_id = "ee_link";
  g.pre_grasp_approach.min_distance = 0.2;
  g.pre_grasp_approach.desired_distance = 0.4;


  grasps.push_back(g);
  group.pick("coke", grasps);
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

int main(int argc, char **argv) {
    //cout << argv[0] << endl;
    ros::init(argc, argv,"moveit_group_goal");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nodeHandle;

    moveit::planning_interface::MoveGroup group("manipulator");
    group.setMaxVelocityScalingFactor(0.1);
    group.setMaxAccelerationScalingFactor(0.1);
    group.setPlanningTime(10.0);
    group.setNumPlanningAttempts(100);
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setPoseReferenceFrame("world");
	group.setGoalTolerance(0.001);

    group.setStartStateToCurrentState();

    ROS_INFO("End effector reference frame: %s", group.getEndEffectorLink().c_str());
    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id="world";
    target_pose.header.stamp=ros::Time::now()+ros::Duration(10.0);

    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface plan_scene;
    std::map<std::string,geometry_msgs::Pose> mymap;
    std::vector<std::string> obj;
    obj.push_back("coke1");  
    mymap = plan_scene.getObjectPoses(obj);
    auto search = mymap.find("coke1");
    if(search != mymap.end()) {
        target_pose.pose.position.x = search->second.position.x + 0.0325;
	    target_pose.pose.position.y = search->second.position.y + 0.0325;
	    target_pose.pose.position.z = search->second.position.z + 0.3;
        ROS_INFO("Object position: %lf %lf %lf",search->second.position.x,search->second.position.y,search->second.position.z);
    }
    else {
        ROS_ERROR("Not found coke");
        exit(1);
    }
    //ROS_ERROR("%s-%f %s-%f %s-%f",argv[1],atof(argv[1]),argv[2],atof(argv[2]),argv[3],atof(argv[3]));
    target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-M_PI/2,M_PI/2,0); //horizontal
    //target_pose.pose.orientation.x = 0.5;
    
    //moveit_msgs::Constraints constraint;
    //constraint 
    //open_gripper();
    sleep(2);
    group.setPoseTarget(target_pose);
    //group.setPathConstraints(constraint);
    bool goal_reached = false;
    int loop = 0;
    bool success = false;
    moveit::planning_interface::MoveGroup::Plan my_plan;
   while(ros::ok() && !goal_reached && loop < 10){
        success = group.plan(my_plan);
        ROS_INFO("plan: %s",success?"SUCCESS":"FAILED");
        if(success) {
            char q;
            std::cout << "Please make sure that your robot can move freely between these poses before proceeding!\n";
            std::cout << "Continue? y/n: ";
            std::cin >> q;
            std::cout << endl;
            if(q == 'y'){
               ROS_INFO("Moving...");
                //group.stop();
                group.execute(my_plan);
                goal_reached = true;
            }
        }
        loop++;
    }
    if(loop == 10){
        ROS_ERROR("Fail found a plan");
        exit(1);
    }
    sleep(5);

    //close_gripper();
    move_group.attachObject("coke1");
    sleep(3);

    goal_reached = false;
    loop = 0;
    target_pose.pose.position.x = 0.2;
    target_pose.pose.position.y = 0.2;
    target_pose.pose.position.z = 1.2;
    //move_group.place("coke",target_pose);
    move_group.detachObject("coke1");
    group.setPoseTarget(target_pose);
    while(ros::ok() && !goal_reached && loop < 10){
        success = group.plan(my_plan);
        ROS_INFO("plan: %s",success?"SUCCESS":"FAILED");
        if(success) {
            char q;
            std::cout << "Please make sure that your robot can move freely between these poses before proceeding!\n";
            std::cout << "Continue? y/n: ";
            std::cin >> q;
            std::cout << endl;
            if(q == 'y'){
               ROS_INFO("Moving...");
                //group.stop();
                group.execute(my_plan);
                goal_reached = true;
            }
        }
        loop++;
    }
    //move_group.detachObject("coke1");
    return 0;
}