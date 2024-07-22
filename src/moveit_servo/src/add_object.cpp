#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "add_collision_object");
    ros::NodeHandle nh;

    ros::AsyncSpinner spin(1);
    spin.start();

    moveit::planning_interface::PlanningSceneInterface current_scene;
    sleep(5.0);

    moveit_msgs::CollisionObject cylinder;
    cylinder.id = "cobot_cylinder";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.6;
    primitive.dimensions[1] = 0.2;
    primitive.dimensions[2] = 0.2;

    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = 0.0;
    pose.position.y = -1;
    pose.position.z = 0.4;

    cylinder.primitives.push_back(primitive);
    cylinder.primitive_poses.push_back(pose);
    cylinder.operation = cylinder.ADD;
        cylinder.header.frame_id = "base_link";
    
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(cylinder);

    current_scene.addCollisionObjects(collision_objects);
        sleep(2);

    ros::shutdown();

    return 0;
}