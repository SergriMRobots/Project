#ifndef COMMAND_VALIDATOR_H
#define COMMAND_VALIDATOR_H
#include <moveit/robot_state/robot_state.h>
#include <ros/ros.h>

#include <kdl/chain.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "moveit_servo/servo_parameters.h"
namespace moveit_servo
{
class CommandValidator
{
  std::string m_joint_group_name;
  KDL::Chain m_manipulator;
  double m_publish_period;

  KDL::JntArrayVel toKDL(moveit::core::RobotStatePtr current_state_);
  KDL::Twist toKDL(const Eigen::VectorXd& command_twist);

public:
  bool init(const moveit::core::RobotStatePtr& robot_state_ptr, const ServoParameters& parameters);
  bool commandValidateControl(moveit::core::RobotStatePtr current_state_, const Eigen::VectorXd& delta_x,
                              Eigen::ArrayXd& delta_theta_);
};

}  // namespace moveit_servo
#endif  // KDL_EXTENSION_H
