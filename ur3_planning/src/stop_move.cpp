#include <ros/ros.h>
#include <iostream>
#include <string>
#include <stdlib.h>     /* atof */
#include <unistd.h>
#include <moveit/move_group_interface/move_group.h>



using namespace std;

int main(int argc, char **argv) {
    
    ros::init(argc, argv,"moveit_group_stop");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nodeHandle;

    moveit::planning_interface::MoveGroup group("manipulator");
    group.stop();
    sleep(3);
    return 0;
} 
