#ifndef TRAJECTORY_PUBLISHER_H
#define TRAJECTORY_PUBLISHER_H
#include "command_publisher_interface.h"
namespace moveit_servo
{
class TrajectoryPublisher : public CommandPublisherInterface
{
public:
  TrajectoryPublisher(ros::NodeHandle& nh_, const std::string& output_topic);
  void publish(trajectory_msgs::JointTrajectory& traj) override;
};
}  // namespace moveit_servo
#endif  // TRAJECTORY_PUBLISHER_H
