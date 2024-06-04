#include <moveit_servo/servo_calcs.h>

#define PRINT_VAR(var) '\n' << #var << ": " << var
namespace moveit_servo
{
static const std::string LOGNAME = "cartesian_servo_calcs";
constexpr size_t ROS_LOG_THROTTLE_PERIOD = 30;  // Seconds to throttle logs inside loops
// Perform the servoing calculations
bool ServoCalcs::cartesianServoCalcs(geometry_msgs::TwistStamped& cmd,
                                     trajectory_msgs::JointTrajectory& joint_trajectory)
{
  // Check for nan's in the incoming command
  if (std::isnan(cmd.twist.linear.x) || std::isnan(cmd.twist.linear.y) || std::isnan(cmd.twist.linear.z) ||
      std::isnan(cmd.twist.angular.x) || std::isnan(cmd.twist.angular.y) || std::isnan(cmd.twist.angular.z))
  {
    ROS_WARN_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME,
                                   "nan in incoming command. Skipping this datapoint.");
    return false;
  }

  // If incoming commands should be in the range [-1:1], check for |delta|>1
  if (parameters_.command_in_type == CommandInType::unitless)
  {
    if ((fabs(cmd.twist.linear.x) > 1) || (fabs(cmd.twist.linear.y) > 1) || (fabs(cmd.twist.linear.z) > 1) ||
        (fabs(cmd.twist.angular.x) > 1) || (fabs(cmd.twist.angular.y) > 1) || (fabs(cmd.twist.angular.z) > 1))
    {
      ROS_WARN_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME,
                                     "Component of incoming command is >1. Skipping this datapoint.");
      return false;
    }
  }

  // Set uncontrolled dimensions to 0 in command frame
  if (!control_dimensions_[0])
    cmd.twist.linear.x = 0;
  if (!control_dimensions_[1])
    cmd.twist.linear.y = 0;
  if (!control_dimensions_[2])
    cmd.twist.linear.z = 0;
  if (!control_dimensions_[3])
    cmd.twist.angular.x = 0;
  if (!control_dimensions_[4])
    cmd.twist.angular.y = 0;
  if (!control_dimensions_[5])
    cmd.twist.angular.z = 0;

  // Transform the command to the MoveGroup planning frame
  if (cmd.header.frame_id != parameters_.planning_frame)
  {
    Eigen::Vector3d translation_vector(cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.linear.z);
    Eigen::Vector3d angular_vector(cmd.twist.angular.x, cmd.twist.angular.y, cmd.twist.angular.z);

    // If the incoming frame is empty or is the command frame, we use the
    // previously calculated tf
    if (cmd.header.frame_id.empty() || cmd.header.frame_id == parameters_.robot_link_command_frame)
    {
      translation_vector = tf_moveit_to_robot_cmd_frame_.linear() * translation_vector;
      angular_vector = tf_moveit_to_robot_cmd_frame_.linear() * angular_vector;
    }
    else if (cmd.header.frame_id == parameters_.ee_frame_name)
    {
      // If the frame is the EE frame, we already have that transform as well
      translation_vector = tf_moveit_to_ee_frame_.linear() * translation_vector;
      angular_vector = tf_moveit_to_ee_frame_.linear() * angular_vector;
    }
    else
    {
      // We solve (planning_frame -> base -> cmd.header.frame_id)
      // by computing (base->planning_frame)^-1 * (base->cmd.header.frame_id)
      const auto tf_moveit_to_incoming_cmd_frame =
          current_state_->getGlobalLinkTransform(parameters_.planning_frame).inverse() *
          current_state_->getGlobalLinkTransform(cmd.header.frame_id);

      translation_vector = tf_moveit_to_incoming_cmd_frame.linear() * translation_vector;
      angular_vector = tf_moveit_to_incoming_cmd_frame.linear() * angular_vector;
    }

    // Put these components back into a TwistStamped
    cmd.header.frame_id = parameters_.planning_frame;
    cmd.twist.linear.x = translation_vector(0);
    cmd.twist.linear.y = translation_vector(1);
    cmd.twist.linear.z = translation_vector(2);
    cmd.twist.angular.x = angular_vector(0);
    cmd.twist.angular.y = angular_vector(1);
    cmd.twist.angular.z = angular_vector(2);
  }

  Eigen::VectorXd delta_x = scaleCartesianCommand(cmd);
  // Convert from cartesian commands to joint commands
  Eigen::MatrixXd jacobian = current_state_->getJacobian(joint_model_group_);

  // May allow some dimensions to drift, based on drift_dimensions
  // i.e. take advantage of task redundancy.
  // Remove the Jacobian rows corresponding to True in the vector
  // drift_dimensions Work backwards through the 6-vector so indices don't get
  // out of order
  for (auto dimension = jacobian.rows() - 1; dimension >= 0; --dimension)
  {
    if (drift_dimensions_[dimension] && jacobian.rows() > 1)
    {
      removeDimension(jacobian, delta_x, dimension);
    }
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd =
      Eigen::JacobiSVD<Eigen::MatrixXd>(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::MatrixXd matrix_s = svd.singularValues().asDiagonal();
  Eigen::MatrixXd pseudo_inverse = svd.matrixV() * matrix_s.inverse() * svd.matrixU().transpose();

  Eigen::ArrayXd pinv_solve = pseudo_inverse * delta_x;

  Eigen::VectorXd dq_solve = (pinv_solve / parameters_.publish_period) * KDL::rad2deg;
  m_cmd_validator.commandValidateControl(current_state_, delta_x, pinv_solve);
  Eigen::VectorXd afterCommandValidator = (pinv_solve / parameters_.publish_period) * KDL::rad2deg;

  auto changeOrder = [this](const Eigen::ArrayXd& pinv_solve) -> Eigen::ArrayXd {
    Eigen::VectorXd result(getNumJoints());
    result.setZero();
    auto joint_names = joint_model_group_->getActiveJointModelNames();
    if (pinv_solve.size() != static_cast<decltype(pinv_solve.size())>(joint_names.size()))
    {
      return result;
    }
    for (decltype(joint_names.size()) i = 0; i < joint_names.size(); i++)
    {
      auto joint_model_it = joint_model_map.find(joint_names[i]);
      if (joint_model_it == joint_model_map.end())
      {
        result.setZero();
        return result;
      }
      auto index_in_map = std::distance(joint_model_map.begin(), joint_model_it);
      result[index_in_map] = pinv_solve[i];
    }
    return result;
  };
  delta_theta_ = changeOrder(pinv_solve);

  Eigen::VectorXd afterChangeOrder = (delta_theta_ / parameters_.publish_period) * KDL::rad2deg;

  auto vel_scale = enforceVelLimits(delta_theta_);
  Eigen::VectorXd afterEnforceVelLimits = (delta_theta_ / parameters_.publish_period) * KDL::rad2deg;

  auto acc_scale = enforceAccLimits(delta_theta_);
  Eigen::VectorXd afterenforceAccLimits = (delta_theta_ / parameters_.publish_period) * KDL::rad2deg;

  // If close to a collision or a singularity, decelerate
  applyVelocityScaling(delta_theta_, velocityScalingFactorForSingularity(delta_x, svd, pseudo_inverse));
  Eigen::VectorXd afterapplyVelocityScaling = (delta_theta_ / parameters_.publish_period) * KDL::rad2deg;
  // clang-format off
  ROS_DEBUG_STREAM_NAMED("cartesian_compute",
                        std::fixed << std::setprecision(5)
                        << PRINT_VAR(dq_solve.transpose())
                        << PRINT_VAR(afterCommandValidator.transpose())
                        << PRINT_VAR(afterChangeOrder.transpose())
                        << PRINT_VAR(afterEnforceVelLimits.transpose())
                        << PRINT_VAR(afterenforceAccLimits.transpose())
                        << PRINT_VAR(afterapplyVelocityScaling.transpose())
                        << PRINT_VAR(vel_scale)
                        << PRINT_VAR(acc_scale));
  // clang-format on

  prev_joint_velocity_ = delta_theta_ / parameters_.publish_period;
  m_last_calculated = ros::Time::now();

  return convertDeltasToOutgoingCmd(joint_trajectory);
}
}  // namespace moveit_servo
