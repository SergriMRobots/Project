#include <moveit/collision_detection_bullet/collision_env_bullet.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>


#include "moveit_servo/bullet_collision.h"

#include "rosparam_shortcuts/rosparam_shortcuts.h"

#include <moveit/collision_detection/collision_tools.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>

namespace moveit_servo
{

std::ostream& operator<<(std::ostream& os, const collision_detection::AllowedCollisionMatrix& acm)
{
  acm.print(os);
  return os;
}

constexpr double EPSILON = 1e-6;  // For very small numeric comparisons

double BulletCollisionCheck::computeScale(double current_dist)
{
  double warn=0.01;//FIXME
  double stop=0.002;//FIXME 
  if (current_dist > warn)
    return 1.0;
  else if (current_dist <= warn && current_dist > stop)
  {
    double k = 1 / (warn - stop);
    double b = -k * stop;
    return k * current_dist + b;
  }
  else
    return 0.0;
}

BulletCollisionCheck::BulletCollisionCheck(
    ros::NodeHandle& nh, const ServoParameters& parameters,
    const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
  : CollisionCheck(nh, parameters, planning_scene_monitor)
{
  collision_check_type_ = CollisionCheckType::K_BULLET_DISTANCE;
  
}



double BulletCollisionCheck::calcIteration(const sensor_msgs::JointState& now,
                                                 const sensor_msgs::JointState& /*future*/)
{

  ROS_DEBUG_STREAM_NAMED("bullet_tutorial", "Init");

  // этой строчкой мы отключаем дефолтный FCL и меняем его на bullet
  planning_scene_monitor_->getPlanningScene()->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create(),
                                             /* exclusive = */ true);

  collision_detection::CollisionResult res;
  collision_detection::CollisionRequest req;
  // parameters_.move_group_name;


  req.group_name = "manipulator"; //parameters_.move_group_name; //FIXME

  // req.contacts = true;
  req.distance=true;
  res.clear();
  state.setVariableValues(now);
  planning_scene_monitor_->getPlanningScene()->checkCollision(req, res, state);
  
  // state.setVariableValues(now);
  // updateJoints();
  auto current_dist = res.distance;

  auto current_time = ros::Time::now();
  double scale = computeScale(current_dist);
  return scale;
  // return current_dist; //FIXME

}
}  // namespace moveit_servo
