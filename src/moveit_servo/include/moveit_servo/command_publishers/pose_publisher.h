#ifndef POSE_PUBLISHER_H
#define POSE_PUBLISHER_H
#include <geometry_msgs/PoseStamped.h>
#include <kdl_conversions/kdl_msg.h>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "command_publisher_interface.h"
namespace moveit_servo
{
class PosePublisher : public CommandPublisherInterface
{
  KDL::Chain chain;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
  geometry_msgs::PoseStamped msg;
  std::vector<std::string> joints_order;

public:
  PosePublisher(ros::NodeHandle& nh_, const std::string& output_topic, const std::vector<std::string>& joints_order);

  void publish(trajectory_msgs::JointTrajectory& traj) override;
};

}  // namespace moveit_servo
#endif  // POSE_PUBLISHER_H
