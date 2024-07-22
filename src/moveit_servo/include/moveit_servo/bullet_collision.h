#ifndef BULLET_COLLISION_H
#define BULLET_COLLISION_H
#include <std_msgs/String.h>
#include<visualization_msgs/MarkerArray.h>
#include "collision_check.h"
namespace moveit_servo
{
using LinkPair = std::pair<std::string, std::string>;
class BulletCollisionCheck : public CollisionCheck
{
    enum class CoefType
  {
    SELF,
    SCENE
  };

    struct ScaleResult
  {
    double dist; /**< Минимальная дистанция между линками*/
    double scale; /**< Рассчитанный коэффициент замедленния для этой пары линков*/
    LinkPair pair; /**< Пара линков */
    CoefType type; /**< Тип столкновения этой пары линков*/
  };
  double current_collision_distance_ = 0;
 
public:

  double calcIteration(const sensor_msgs::JointState& now, const sensor_msgs::JointState& future) override;

  BulletCollisionCheck(ros::NodeHandle& nh, const moveit_servo::ServoParameters& parameters,
                                  const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor);
  friend std::ostream& operator<<(std::ostream& os, const std::vector<ScaleResult>& computed_coef);

private: 
  double computeScale(double current_dist);

};

}  // namespace moveit_servo
#endif  // THRESHOLD_DISTANCE_CHECK_H
