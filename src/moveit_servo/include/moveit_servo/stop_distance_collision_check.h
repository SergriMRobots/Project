#ifndef STOP_DISTANCE_COLLISION_CHECK_H
#define STOP_DISTANCE_COLLISION_CHECK_H
#include "collision_check.h"
namespace moveit_servo
{
class StopDistanceCollisionCheck : public CollisionCheck
{
  struct StopDistanceParameters
  {
    double collision_distance_safety_factor;
    double min_allowable_collision_distance;
    bool readParams(ros::NodeHandle& nh);
  } stop_distance_params;
  ros::Time last_time_called;

  // Variables for stop-distance-based collision checking
  double current_collision_distance_ = 0;
  double derivative_of_collision_distance_ = 0;
  double prev_collision_distance_ = 0;
  double est_time_to_collision_ = 0;
  double worst_case_stop_time_ = std::numeric_limits<double>::max();

public:
  StopDistanceCollisionCheck(ros::NodeHandle& nh, const ServoParameters& parameters,
                             const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor);
  double calcIteration(const sensor_msgs::JointState& now, const sensor_msgs::JointState&) override;
  /** \brief Callback for stopping time, from the thread that is aware of
   * velocity and acceleration */
  void setWorstCaseStop(double new_worst_case_stop_time_);
};
}  // namespace moveit_servo

#endif  // STOP_DISTANCE_COLLISION_CHECK_H
