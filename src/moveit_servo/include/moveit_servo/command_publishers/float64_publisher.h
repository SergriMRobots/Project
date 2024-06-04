#ifndef FLOAT64_PUBLISHER_H
#define FLOAT64_PUBLISHER_H
#include <std_msgs/Float64MultiArray.h>

#include "command_publisher_interface.h"
namespace moveit_servo
{
class Float64Publisher : public CommandPublisherInterface
{
protected:
  std_msgs::Float64MultiArray msg;
  std::vector<std::string> joints_order;

public:
  Float64Publisher(ros::NodeHandle& nh_, const std::string& output_topic, const std::vector<std::string>& joints_order);
};

class Float64VelocityPublisher : public Float64Publisher
{
public:
  Float64VelocityPublisher(ros::NodeHandle& nh_, const std::string& output_topic,
                           const std::vector<std::string>& joints_order);

  void publish(trajectory_msgs::JointTrajectory& traj) override;
};
class Float64PositionPublisher : public Float64Publisher
{
public:
  Float64PositionPublisher(ros::NodeHandle& nh_, const std::string& output_topic,
                           const std::vector<std::string>& joints_order);
  void publish(trajectory_msgs::JointTrajectory& traj) override;
};
}  // namespace moveit_servo
#endif  // FLOAT64_PUBLISHER_H
