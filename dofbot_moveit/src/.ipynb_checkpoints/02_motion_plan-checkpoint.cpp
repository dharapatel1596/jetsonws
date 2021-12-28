#include <iostream>
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/LinearMath/Quaternion.h>

using namespace std;

const float DE2RA = M_PI / 180.0f;

int main(int argc, char **argv) {

    ros::init(argc, argv, "dofbot_motion_plan_cpp");

    ros::NodeHandle n;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface dofbot("dofbot");

    dofbot.allowReplanning(true);

    dofbot.setPlanningTime(5);

    dofbot.setNumPlanningAttempts(10);

    dofbot.setGoalPositionTolerance(0.01);

    dofbot.setGoalOrientationTolerance(0.01);

    dofbot.setMaxVelocityScalingFactor(1.0);

    dofbot.setMaxAccelerationScalingFactor(1.0);

    geometry_msgs::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.05957241;
    pose.position.z = 0.1680498;

    tf::Quaternion quaternion;

    double Roll = -140;
    double Pitch = 0.0;
    double Yaw = 0.0;

    quaternion.setRPY(Roll * DE2RA, Pitch * DE2RA, Yaw * DE2RA);
//    pose.orientation.x = 0.940132;
//    pose.orientation.y = -0.000217502;
//    pose.orientation.z = 0.000375234;
//    pose.orientation.w = -0.340811;
    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();
    string link = dofbot.getEndEffectorLink();

    dofbot.setPoseTarget(pose, link);
    int index = 0;

    while (index <= 10) {
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        const moveit::planning_interface::MoveItErrorCode &code = dofbot.plan(plan);
        if (code == code.SUCCESS) {
            ROS_INFO_STREAM("plan success");
            dofbot.execute(plan);
            break;
        } else {
            ROS_INFO_STREAM("plan error");
        }
        index++;
    }
    return 0;
}

