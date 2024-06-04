/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*      Title     : servo.cpp
 *      Project   : moveit_servo
 *      Created   : 3/9/2017
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
 */

#include <angles/angles.h>
#include <moveit_servo/make_shared_from_pool.h>
#include <moveit_servo/servo.h>

namespace moveit_servo
{
namespace
{
const std::string LOGNAME = "servo_node";
constexpr double ROBOT_STATE_WAIT_TIME = 10.0;  // seconds
}  // namespace

Servo::Servo(ros::NodeHandle& nh, const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
  : nh_(nh), planning_scene_monitor_(planning_scene_monitor)
{
  // Read ROS parameters, typically from YAML file
  if (!readParameters())
    exit(EXIT_FAILURE);

  // Async spinner is needed to receive messages to wait for the robot state to
  // be complete
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Confirm the planning scene monitor is ready to be used
  if (!planning_scene_monitor_->getStateMonitor())
  {
    planning_scene_monitor_->startStateMonitor(parameters_.joint_topic);
  }
  planning_scene_monitor->getStateMonitor()->enableCopyDynamics(true);

  if (!planning_scene_monitor_->getStateMonitor()->waitForCompleteState(parameters_.move_group_name,
                                                                        ROBOT_STATE_WAIT_TIME))
  {
    ROS_FATAL_NAMED(LOGNAME, "Timeout waiting for current state");
    exit(EXIT_FAILURE);
  }

  servo_calcs_ = std::make_unique<ServoCalcs>(nh_, parameters_, planning_scene_monitor_);
}

// Read ROS parameters, typically from YAML file
bool Servo::readParameters()
{
  std::size_t error = 0;

  // Optional parameter sub-namespace specified in the launch file. All other
  // parameters will be read from this namespace.
  std::string parameter_ns;
  ros::param::get("~parameter_ns", parameter_ns);

  // If parameters have been loaded into sub-namespace within the node
  // namespace, append the parameter namespace to load the parameters correctly.
  ros::NodeHandle nh = parameter_ns.empty() ? nh_ : ros::NodeHandle(nh_, parameter_ns);

  error += !rosparam_shortcuts::get(LOGNAME, nh, "publish_period_hz", parameters_.publish_period);
  parameters_.publish_period = convertToROS(parameters_.publish_period, InputValueType::HERZ);

  error += !rosparam_shortcuts::get(LOGNAME, nh, "num_outgoing_halt_msgs_to_publish",
                                    parameters_.num_outgoing_halt_msgs_to_publish);
  error += !rosparam_shortcuts::get(LOGNAME, nh_, "robot_link_command_frame", parameters_.robot_link_command_frame);
  error += !rosparam_shortcuts::get(LOGNAME, nh, "scale/linear", parameters_.linear_scale);
  error += !rosparam_shortcuts::get(LOGNAME, nh, "scale/rotational_deg", parameters_.rotational_scale);
  parameters_.rotational_scale = convertToROS(parameters_.rotational_scale, InputValueType::DEGREE);
  error += !rosparam_shortcuts::get(LOGNAME, nh, "scale/default_joint_deg", parameters_.default_joint_scale);
  parameters_.default_joint_scale = convertToROS(parameters_.default_joint_scale, InputValueType::DEGREE);

  error += !rosparam_shortcuts::get(LOGNAME, nh, "low_pass_filter_coeff", parameters_.low_pass_filter_coeff);
  error += !rosparam_shortcuts::get(LOGNAME, nh, "joint_topic", parameters_.joint_topic);
  std::string command_in_type;
  error += !rosparam_shortcuts::get(LOGNAME, nh, "command_in_type", command_in_type);
  error += !rosparam_shortcuts::get(LOGNAME, nh, "cartesian_command_in_topic", parameters_.cartesian_command_in_topic);
  error += !rosparam_shortcuts::get(LOGNAME, nh, "joint_command_in_topic", parameters_.joint_command_in_topic);
  error += !rosparam_shortcuts::get(LOGNAME, nh, "incoming_command_timeout", parameters_.incoming_command_timeout);
  error +=
      !rosparam_shortcuts::get(LOGNAME, nh, "lower_singularity_threshold", parameters_.lower_singularity_threshold);
  error += !rosparam_shortcuts::get(LOGNAME, nh, "hard_stop_singularity_threshold",
                                    parameters_.hard_stop_singularity_threshold);
  error += !rosparam_shortcuts::get(LOGNAME, nh_, "command_out_topic", parameters_.command_out_topic);
  error += !rosparam_shortcuts::get(LOGNAME, nh, "move_group_name", parameters_.move_group_name);
  error += !rosparam_shortcuts::get(LOGNAME, nh, "planning_frame", parameters_.planning_frame);
  error += !rosparam_shortcuts::get(LOGNAME, nh, "ee_frame_name", parameters_.ee_frame_name);
  error += !rosparam_shortcuts::get(LOGNAME, nh_, "use_gazebo", parameters_.use_gazebo);
  error += !rosparam_shortcuts::get(LOGNAME, nh, "joint_limit_margin_deg", parameters_.joint_limit_margin);
  parameters_.joint_limit_margin = convertToROS(parameters_.joint_limit_margin, InputValueType::DEGREE);
  error += !rosparam_shortcuts::get(LOGNAME, nh, "command_out_type", parameters_.command_out_type);
  error += !rosparam_shortcuts::get(LOGNAME, nh, "publish_joint_positions", parameters_.publish_joint_positions);
  error += !rosparam_shortcuts::get(LOGNAME, nh, "publish_joint_velocities", parameters_.publish_joint_velocities);

  // Parameters for collision checking
  error += !rosparam_shortcuts::get(LOGNAME, nh, "check_collisions", parameters_.check_collisions);
  error += !rosparam_shortcuts::get(LOGNAME, nh, "collision_check_type", parameters_.collision_check_type);
  error += !rosparam_shortcuts::get(LOGNAME, nh, "collisions_switch_service", parameters_.collisions_switch_service);

  // This parameter name was changed recently.
  // Try retrieving from the correct name. If it fails, then try the deprecated
  // name.
  // TODO(andyz): remove this deprecation warning in ROS Noetic
  if (!rosparam_shortcuts::get(LOGNAME, nh, "status_topic", parameters_.status_topic))
  {
    ROS_WARN_NAMED(LOGNAME, "'status_topic' parameter is missing. Recently renamed from "
                            "'warning_topic'. Please update "
                            "the servoing yaml file.");
    error += !rosparam_shortcuts::get(LOGNAME, nh, "warning_topic", parameters_.status_topic);
  }

  if (nh.hasParam("low_latency_mode"))
  {
    error += !rosparam_shortcuts::get(LOGNAME, nh, "low_latency_mode", parameters_.low_latency_mode);
  }
  else
  {
    ROS_WARN_NAMED(LOGNAME, "'low_latency_mode' is a new parameter that runs servo calc "
                            "immediately after receiving "
                            "input.  Setting to the default value of false.");
    parameters_.low_latency_mode = false;
  }

  rosparam_shortcuts::shutdownIfError(LOGNAME, error);

  // Input checking
  if (parameters_.publish_period <= 0.)
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter 'publish_period' should be "
                            "greater than zero. Check yaml file.");
    return false;
  }
  if (parameters_.num_outgoing_halt_msgs_to_publish < 0)
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter 'num_outgoing_halt_msgs_to_publish' should be "
                            "greater than zero. Check yaml file.");
    return false;
  }
  if (parameters_.hard_stop_singularity_threshold < parameters_.lower_singularity_threshold)
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter 'hard_stop_singularity_threshold' "
                            "should be greater than 'lower_singularity_threshold.' "
                            "Check yaml file.");
    return false;
  }
  if ((parameters_.hard_stop_singularity_threshold < 0.) || (parameters_.lower_singularity_threshold < 0.))
  {
    ROS_WARN_NAMED(LOGNAME, "Parameters 'hard_stop_singularity_threshold' "
                            "and 'lower_singularity_threshold' should be "
                            "greater than zero. Check yaml file.");
    return false;
  }
  if (parameters_.low_pass_filter_coeff < 0.)
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter 'low_pass_filter_coeff' should be "
                            "greater than zero. Check yaml file.");
    return false;
  }
  if (parameters_.joint_limit_margin < 0.)
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter 'joint_limit_margin' should be "
                            "greater than or equal to zero. Check yaml file.");
    return false;
  }
  if (command_in_type == "unitless")
    parameters_.command_in_type = CommandInType::unitless;
  else if (command_in_type == "speed_units")
    parameters_.command_in_type = CommandInType::speed_units;
  else
  {
    ROS_WARN_NAMED(LOGNAME, "command_in_type should be 'unitless' or "
                            "'speed_units'. Check yaml file.");
    return false;
  }
  if (parameters_.command_out_type != "trajectory_msgs/JointTrajectory" &&
      parameters_.command_out_type != "std_msgs/Float64MultiArray" &&
      parameters_.command_out_type != "geometry_msgs/PoseStamped")
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter command_out_type should be "
                            "'trajectory_msgs/JointTrajectory' or "
                            "'std_msgs/Float64MultiArray' or "
                            "'geometry_msgs/PoseStamped'. Check yaml file.");
    return false;
  }
  if (!parameters_.publish_joint_positions && !parameters_.publish_joint_velocities)
  {
    ROS_WARN_NAMED(LOGNAME, "At least one of publish_joint_positions / "
                            "publish_joint_velocities / "
                            "Check yaml file.");
    return false;
  }
  if ((parameters_.command_out_type == "std_msgs/Float64MultiArray") && parameters_.publish_joint_positions &&
      parameters_.publish_joint_velocities)
  {
    ROS_WARN_NAMED(LOGNAME, "When publishing a std_msgs/Float64MultiArray, "
                            "you must select positions OR velocities.");
    return false;
  }
  // Collision checking
  if (parameters_.collision_check_type != "threshold_distance" && parameters_.collision_check_type != "stop_distance")
  {
    ROS_WARN_NAMED(LOGNAME, "collision_check_type must be 'threshold_distance' or 'stop_distance'");
    return false;
  }
  return true;
}

void Servo::start()
{
  setPaused(false);
  servo_calcs_->start();
}

Servo::~Servo()
{
  setPaused(true);
}

void Servo::setPaused(bool paused)
{
  servo_calcs_->setPaused(paused);
}

bool Servo::getEEFrameTransform(Eigen::Isometry3d& transform)
{
  return servo_calcs_->getEEFrameTransform(transform);
}

bool Servo::getEEFrameTransform(geometry_msgs::TransformStamped& transform)
{
  return servo_calcs_->getEEFrameTransform(transform);
}

const ServoParameters& Servo::getParameters() const
{
  return parameters_;
}

}  // namespace moveit_servo
