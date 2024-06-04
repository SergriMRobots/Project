#include "moveit_servo/stop_distance_collision_check.h"
#include "rosparam_shortcuts/rosparam_shortcuts.h"
namespace moveit_servo
{
constexpr double EPSILON = 1e-6;  // For very small numeric comparisons

StopDistanceCollisionCheck::StopDistanceCollisionCheck(
    ros::NodeHandle& nh, const ServoParameters& parameters,
    const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
  : CollisionCheck(nh, parameters, planning_scene_monitor)
{
  collision_check_type_ = CollisionCheckType::K_STOP_DISTANCE;
  if (!stop_distance_params.readParams(nh))
    exit(EXIT_FAILURE);
}

double StopDistanceCollisionCheck::calcIteration(const sensor_msgs::JointState& now,
                                                 const sensor_msgs::JointState& /*future*/)
{
  // Retrieve the worst-case time to stop, based on current joint velocities

  // Calculate rate of change of distance to nearest collision
  if (last_time_called.toSec() == 0.0)
  {
    last_time_called = ros::Time::now();
    return 1.0;
  }
  // Update to the latest current state
  state.setVariableValues(now);
  auto current_dist = computeCollisionResultForState(state);
  current_collision_distance_ = current_dist.minimalCollistionDist();

  auto current_time = ros::Time::now();

  derivative_of_collision_distance_ =
      (current_collision_distance_ - prev_collision_distance_) / (current_time - last_time_called).toSec();
  last_time_called = current_time;
  ROS_DEBUG_NAMED("derivative_of_collision_distance_", "derivative_of_collision_distance_: %f",
                  derivative_of_collision_distance_);
  if (current_collision_distance_ < stop_distance_params.min_allowable_collision_distance &&
      derivative_of_collision_distance_ <= 0)
  {
    return 0;
  }
  // Only bother doing calculations if we are moving toward the nearest
  // collision
  else if (derivative_of_collision_distance_ < -EPSILON)
  {
    // At the rate the collision distance is decreasing, how long until we
    // collide?
    est_time_to_collision_ = fabs(current_collision_distance_ / derivative_of_collision_distance_);

    // halt if we can't stop fast enough (including the safety factor)
    if (est_time_to_collision_ < (stop_distance_params.collision_distance_safety_factor * worst_case_stop_time_))
    {
      return 0.0;
    }
  }

  // Update for the next iteration
  prev_collision_distance_ = current_collision_distance_;
  return 0.0;
}

void StopDistanceCollisionCheck::setWorstCaseStop(double new_worst_case_stop_time_)
{
  worst_case_stop_time_ = new_worst_case_stop_time_;
}

bool StopDistanceCollisionCheck::StopDistanceParameters::readParams(ros::NodeHandle& nh)
{
  std::size_t error = 0;
  error += !rosparam_shortcuts::get("", nh, "collision_distance_safety_factor", collision_distance_safety_factor);
  error += !rosparam_shortcuts::get("", nh, "min_allowable_collision_distance", min_allowable_collision_distance);
  if (error != 0)
    return false;
  if (collision_distance_safety_factor < 1)
  {
    ROS_WARN("Parameter 'collision_distance_safety_factor' should be "
             "greater than or equal to 1. Check yaml file.");
    return false;
  }
  if (min_allowable_collision_distance < 0)
  {
    ROS_WARN("Parameter 'min_allowable_collision_distance' should be "
             "greater than zero. Check yaml file.");
    return false;
  }
  return true;
}

}  // namespace moveit_servo
