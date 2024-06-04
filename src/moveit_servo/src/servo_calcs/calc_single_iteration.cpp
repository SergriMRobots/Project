#include <moveit_servo/servo_calcs.h>
#include <moveit_servo/make_shared_from_pool.h>

namespace moveit_servo
{
static const std::string LOGNAME = "calculate_single_iteration";
constexpr size_t ROS_LOG_THROTTLE_PERIOD = 30;  // Seconds to throttle logs inside loops
void ServoCalcs::calculateSingleIteration()
{
  // Publish status each loop iteration
  auto status_msg = moveit::util::make_shared_from_pool<std_msgs::Int8>();
  status_msg->data = static_cast<int8_t>(status_);
  status_pub_.publish(status_msg);

  // Always update the joints and end-effector transform for 2 reasons:
  // 1) in case the getCommandFrameTransform() method is being used
  // 2) so the low-pass filters are up to date and don't cause a jump
  updateJoints();

  // Update from latest state
  current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();

  if (latest_twist_stamped_)
    twist_stamped_cmd_ = *latest_twist_stamped_;
  if (latest_joint_cmd_)
    joint_servo_cmd_ = *latest_joint_cmd_;

  // Check for stale cmds
  twist_command_is_stale_ =
      ((ros::Time::now() - latest_twist_command_stamp_) >= ros::Duration(parameters_.incoming_command_timeout));
  joint_command_is_stale_ =
      ((ros::Time::now() - latest_joint_command_stamp_) >= ros::Duration(parameters_.incoming_command_timeout));

  have_nonzero_twist_stamped_ = latest_nonzero_twist_stamped_;
  have_nonzero_joint_command_ = latest_nonzero_joint_cmd_;

  // Get the transform from MoveIt planning frame to servoing command frame
  // Calculate this transform to ensure it is available via C++ API
  // We solve (planning_frame -> base -> robot_link_command_frame)
  // by computing (base->planning_frame)^-1 * (base->robot_link_command_frame)
  tf_moveit_to_robot_cmd_frame_ = current_state_->getGlobalLinkTransform(parameters_.planning_frame).inverse() *
                                  current_state_->getGlobalLinkTransform(parameters_.robot_link_command_frame);

  // Calculate the transform from MoveIt planning frame to End Effector frame
  // Calculate this transform to ensure it is available via C++ API
  tf_moveit_to_ee_frame_ = current_state_->getGlobalLinkTransform(parameters_.planning_frame).inverse() *
                           current_state_->getGlobalLinkTransform(parameters_.ee_frame_name);

  have_nonzero_command_ = have_nonzero_twist_stamped_ || have_nonzero_joint_command_;

  // Don't end this function without updating the filters
  updated_filters_ = false;

  // If paused or while waiting for initial servo commands, just keep the
  // low-pass filters up to date with current joints so a jump doesn't occur
  // when restarting
  if (wait_for_servo_commands_ || paused_)
  {
    resetLowPassFilters(original_joint_state_);

    // Check if there are any new commands with valid timestamp
    wait_for_servo_commands_ =
        twist_stamped_cmd_.header.stamp == ros::Time(0.) && joint_servo_cmd_.header.stamp == ros::Time(0.);

    // Early exit
    return;
  }

  // If not waiting for initial command, and not paused.
  // Do servoing calculations only if the robot should move, for efficiency
  // Create new outgoing joint trajectory command message
  auto joint_trajectory = moveit::util::make_shared_from_pool<trajectory_msgs::JointTrajectory>();
  bool sudden_non_zero = false;

  // Prioritize cartesian servoing above joint servoing
  // Only run commands if not stale and nonzero
  if (have_nonzero_twist_stamped_ && !twist_command_is_stale_)
  {
    if (!cartesianServoCalcs(twist_stamped_cmd_, *joint_trajectory))
    {
      resetLowPassFilters(original_joint_state_);
      return;
    }
  }
  else if (have_nonzero_joint_command_ && !joint_command_is_stale_)
  {
    if (!jointServoCalcs(joint_servo_cmd_, *joint_trajectory))
    {
      resetLowPassFilters(original_joint_state_);
      return;
    }
  }
  else
  {
    sudden_non_zero = suddenHalt(*joint_trajectory, false);
    // Joint trajectory is not populated with anything, so set it to the last
    // positions and 0 velocity
    //    *joint_trajectory = *last_sent_command_;
    //    for (auto& point : joint_trajectory->points)
    //    {
    //      point.velocities.assign(point.velocities.size(), 0);
    //    }
  }

  // Print a warning to the user if both are stale
  if (twist_command_is_stale_ && joint_command_is_stale_)
  {
    ROS_WARN_STREAM_THROTTLE_NAMED(10, LOGNAME,
                                   "Stale command. "
                                   "Try a larger 'incoming_command_timeout' parameter?");
  }

  // If we should halt
  if (!have_nonzero_command_)
  {
    //    sudden_non_zero = suddenHalt(*joint_trajectory, false);
    have_nonzero_twist_stamped_ = false;
    have_nonzero_joint_command_ = false;
  }

  // Skip the servoing publication if all inputs have been zero for several
  // cycles in a row. num_outgoing_halt_msgs_to_publish == 0 signifies that we
  // should keep republishing forever.
  if (!have_nonzero_command_ && (parameters_.num_outgoing_halt_msgs_to_publish != 0) &&
      (zero_velocity_count_ > parameters_.num_outgoing_halt_msgs_to_publish))
  {
    ok_to_publish_ = false;
    ROS_DEBUG_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME, "All-zero command. Doing nothing.");
  }
  else
  {
    ok_to_publish_ = true;
  }

  // Store last zero-velocity message flag to prevent superfluous warnings.
  // Cartesian and joint commands must both be zero.
  if (!have_nonzero_command_ && !sudden_non_zero)
  {
    // Avoid overflow
    if (zero_velocity_count_ < std::numeric_limits<int>::max())
      ++zero_velocity_count_;
  }
  else
  {
    zero_velocity_count_ = 0;
  }
  if (ok_to_publish_ && !paused_ && outgoing_cmd_interface_)
  {
    outgoing_cmd_interface_->publish(*joint_trajectory);
    last_sent_command_ = joint_trajectory;
  }

  // Update the filters if we haven't yet
  if (!updated_filters_)
    resetLowPassFilters(original_joint_state_);
}
}  // namespace moveit_servo
