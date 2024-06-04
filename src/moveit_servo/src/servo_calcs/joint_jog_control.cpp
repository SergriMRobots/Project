#include <moveit_servo/servo_calcs.h>
#include <moveit_servo/make_shared_from_pool.h>

namespace moveit_servo
{
static const std::string LOGNAME = "joint_servo_calcs";
constexpr size_t ROS_LOG_THROTTLE_PERIOD = 30;  // Seconds to throttle logs inside loops
namespace
{
// Helper function for detecting zeroed message
bool isNonZero(const control_msgs::JointJog& msg)
{
  bool all_zeros = true;
  for (double delta : msg.velocities)
  {
    all_zeros &= (delta == 0.0);
  };
  return !all_zeros;
}

}  // namespace
void ServoCalcs::jointCmdCB(const control_msgs::JointJogConstPtr& msg)
{
  const std::lock_guard<std::mutex> lock(input_mutex_);
  latest_joint_cmd_ = msg;
  latest_nonzero_joint_cmd_ = isNonZero(*latest_joint_cmd_);

  if (msg->header.stamp != ros::Time(0.))
    latest_joint_command_stamp_ = msg->header.stamp;

  // notify that we have a new input
  new_input_cmd_ = true;
  input_cv_.notify_all();
}

Eigen::VectorXd ServoCalcs::scaleJointCommand(const control_msgs::JointJog& command) const
{
  Eigen::VectorXd result(getNumJoints());

  result.setZero();
  for (std::size_t m = 0; m < command.joint_names.size(); ++m)
  {
    const auto& joint_name = command.joint_names[m];
    auto it = joint_model_map.find(joint_name);
    if (it == joint_model_map.end())
    {
      ROS_WARN_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME, "Ignoring joint " << joint_name);
      continue;
    }
    auto c = std::distance(joint_model_map.begin(), it);
    result[c] = command.velocities[m] * parameters_.publish_period;
    if (parameters_.command_in_type == CommandInType::unitless)
      result[c] *= parameters_.default_joint_scale;
  }
  return result;
}
bool ServoCalcs::jointServoCalcs(const control_msgs::JointJog& cmd, trajectory_msgs::JointTrajectory& joint_trajectory)
{
  if (std::any_of(cmd.velocities.begin(), cmd.velocities.end(), [](const double val) { return std::isnan(val); }))
  {
    ROS_WARN_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME,
                                   "nan in incoming command. Skipping this datapoint.");
    return false;
  }

  // Apply user-defined scaling
  delta_theta_ = scaleJointCommand(cmd);

  ROS_DEBUG_STREAM_NAMED(__FUNCTION__, delta_theta_.transpose());

  enforceVelLimits(delta_theta_);
  enforceAccLimits(delta_theta_);

  // If close to a collision, decelerate
  applyVelocityScaling(delta_theta_, 1.0 /* scaling for singularities -- ignore for joint motions */);

  prev_joint_velocity_ = delta_theta_ / parameters_.publish_period;
  m_last_calculated = ros::Time::now();

  return convertDeltasToOutgoingCmd(joint_trajectory);
}

}  // namespace moveit_servo
