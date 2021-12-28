#include <iostream>
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>

using namespace std;

int main(int argc, char **argv) {

    ros::init(argc, argv, "dofbot_random_move_cpp");

    ros::NodeHandle n;

    moveit::planning_interface::MoveGroupInterface dofbot("dofbot");

//    dofbot.setNamedTarget("down");
//    //开始移动
//    dofbot.move();
//    sleep(0.1);
    while (true){
    	//
    	dofbot.setRandomTarget();
    	//
    	dofbot.move();
    	sleep(0.5);
    }
    return 0;
}
