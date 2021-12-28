#include <iostream>
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/LinearMath/Quaternion.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

using namespace std;

int main(int argc, char **argv) {

    ros::init(argc, argv, "dofbot_attached_object_cpp");

    ros::NodeHandle n;

    ros::AsyncSpinner spinner(1);

    spinner.start();

    moveit::planning_interface::MoveGroupInterface dofbot("dofbot");

    string frame = dofbot.getPlanningFrame();

    moveit::planning_interface::PlanningSceneInterface scene;
    ///////////////////////////////////////////////
    vector<string> object_ids;
    scene.removeCollisionObjects(object_ids);

    vector<moveit_msgs::CollisionObject> objects;

    moveit_msgs::CollisionObject obj;

    obj.id = "obj";
    object_ids.push_back(obj.id);

    obj.operation = obj.ADD;

    obj.header.frame_id = frame;
    shape_msgs::SolidPrimitive primitive;

    primitive.type = primitive.BOX;

    primitive.dimensions.resize(3);

    primitive.dimensions[0] = 0.1;
    primitive.dimensions[1] = 0.2;
    primitive.dimensions[2] = 0.1;
    obj.primitives.push_back(primitive);
    geometry_msgs::Pose pose;

    pose.position.x = 0;
    pose.position.y = 0.2;
    pose.position.z = 0.3;

    tf::Quaternion quaternion;

    double Roll = 0.0;
    double Pitch = 0.0;
    double Yaw = 90.0;

    quaternion.setRPY(Roll * M_PI / 180, Pitch * M_PI / 180, Yaw * M_PI / 180);
    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();

    obj.primitive_poses.push_back(pose);
    objects.push_back(obj);
    ///////////////////////////////////////////////

    std::vector<moveit_msgs::ObjectColor> colors;

    moveit_msgs::ObjectColor color;

    color.id = "obj";

    color.color.r = 0;
    color.color.g = 1.0;
    color.color.b = 0;
    color.color.a = 0.5;
    colors.push_back(color);

    scene.applyCollisionObjects(objects, colors);

    ros::waitForShutdown();
    return 0;
}

