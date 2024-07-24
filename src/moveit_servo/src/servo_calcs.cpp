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

/*      Title     : servo_calcs.cpp
 *      Project   : moveit_servo
 *      Created   : 1/11/2019
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
 */

#include <kdl/utilities/utility.h>
#include <moveit_servo/make_shared_from_pool.h>
#include <moveit_servo/servo_calcs.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <ios>
#include <thread>

#include "Eigen/src/Core/Array.h"
#include "Eigen/src/Core/Matrix.h"
#include "moveit_servo/3rdparty_header/expected.hpp"
#include "moveit_servo/command_publishers/float64_publisher.h"
#include "moveit_servo/command_publishers/pose_publisher.h"
#include "moveit_servo/command_publishers/trajectory_publisher.h"
#include "ros/time.h"

// ROS Msgs
#include <moveit_servo/AngularSpeedScaleConfig.h>
#include <moveit_servo/LinearSpeedScaleConfig.h>
// Dynamic parameter server
#include <dynamic_reconfigure/server.h>

#define PRINT_VAR(var) '\n' << #var << ": " << var

namespace moveit_servo
{
static const std::string LOGNAME = "servo_calcs";
constexpr size_t ROS_LOG_THROTTLE_PERIOD = 30;  // Seconds to throttle logs inside loops
std::ostream& operator<<(std::ostream& os, const std::vector<std::string>& joint_names)
{
  os << "\n";
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    os << "Joint[" << i << "]: " << joint_names[i] << "\n";
  }
  return os;
}
namespace
{
// Helper function for detecting zeroed message
bool isNonZero(const geometry_msgs::TwistStamped& msg)
{
  return msg.twist.linear.x != 0.0 || msg.twist.linear.y != 0.0 || msg.twist.linear.z != 0.0 ||
         msg.twist.angular.x != 0.0 || msg.twist.angular.y != 0.0 || msg.twist.angular.z != 0.0;
}

// Helper function for converting Eigen::Isometry3d to
// geometry_msgs/TransformStamped
geometry_msgs::TransformStamped convertIsometryToTransform(const Eigen::Isometry3d& eigen_tf,
                                                           const std::string& parent_frame,
                                                           const std::string& child_frame)
{
  geometry_msgs::TransformStamped output = tf2::eigenToTransform(eigen_tf);
  output.header.frame_id = parent_frame;
  output.child_frame_id = child_frame;

  return output;
}
}  // namespace

// Constructor for the class that handles servoing calculations
void ServoCalcs::linearScaleReconfigureCallback(LinearScaleConfig& config, uint32_t level)
{
  linear_scale.store(std::clamp(config.linear_scale, 0.0, 1.0));
  ROS_DEBUG_NAMED("linear_callback", "New linear scale: %f", linear_scale.load());
}

void ServoCalcs::angularScaleReconfigureCallback(AngularScaleConfig& config, uint32_t level)
{
  angular_scale.store(std::clamp(config.angular_scale, 0.0, 1.0));
  ROS_DEBUG_NAMED("angular_callback", "New angular scale: %f", angular_scale.load());
}

ServoCalcs::ServoCalcs(ros::NodeHandle& nh, ServoParameters& parameters,
                       const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
  : nh_(nh)
  , parameters_(parameters)
  , planning_scene_monitor_(planning_scene_monitor)
  , stop_requested_(true)
  , paused_(false)
  , m_last_calculated(0)
{
  // MoveIt Setup
  current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  joint_model_group_ = current_state_->getJointModelGroup(parameters_.move_group_name);
  for (auto& servo_joint : joint_model_group_->getActiveJointModels())
  {
    JointServo servo{ servo_joint, std::make_shared<LowPassFilter>(parameters_.low_pass_filter_coeff) };
    std::pair<std::string, JointServo> pair(servo_joint->getName(), servo);
    joint_model_map.emplace(pair);
  }
  prev_joint_velocity_ = Eigen::ArrayXd::Zero(joint_model_group_->getActiveJointModels().size());

  // Subscribe to command topics
  twist_stamped_sub_ =
      nh_.subscribe(parameters_.cartesian_command_in_topic, ROS_QUEUE_SIZE, &ServoCalcs::twistStampedCB, this);
  joint_cmd_sub_ = nh_.subscribe(parameters_.joint_command_in_topic, ROS_QUEUE_SIZE, &ServoCalcs::jointCmdCB, this);

  // ROS Server for allowing drift in some dimensions
  drift_dimensions_server_ = nh_.advertiseService(ros::names::append(nh_.getNamespace(), "change_drift_dimensions"),
                                                  &ServoCalcs::changeDriftDimensions, this);

  // ROS Server for changing the control dimensions
  control_dimensions_server_ = nh_.advertiseService(ros::names::append(nh_.getNamespace(), "change_control_dimensions"),
                                                    &ServoCalcs::changeControlDimensions, this);

  // ROS Server to reset the status, e.g. so the arm can move again after a
  // collision
  reset_servo_status_ = nh_.advertiseService(ros::names::append(nh_.getNamespace(), "reset_servo_status"),
                                             &ServoCalcs::resetServoStatus, this);

  // Publish and Subscribe to internal namespace topics
  ros::NodeHandle internal_nh(nh_, "internal");
  collision_velocity_scale_sub_ =
      internal_nh.subscribe("collision_velocity_scale", ROS_QUEUE_SIZE, &ServoCalcs::collisionVelocityScaleCB, this);
  worst_case_stop_time_pub_ = internal_nh.advertise<std_msgs::Float64>("worst_case_stop_time", ROS_QUEUE_SIZE);

  // Publish freshly-calculated joints to the robot.
  // Put the outgoing msg in the right format (trajectory_msgs/JointTrajectory
  // or std_msgs/Float64MultiArray).
  auto active_joint_names = joint_model_group_->getActiveJointModelNames();

  if (parameters_.command_out_type == "trajectory_msgs/JointTrajectory")
    outgoing_cmd_interface_ = std::make_unique<TrajectoryPublisher>(nh_, parameters_.command_out_topic);
  else if (parameters_.command_out_type == "std_msgs/Float64MultiArray")
  {
    if (parameters_.publish_joint_positions)
    {
      outgoing_cmd_interface_ =
          std::make_unique<Float64PositionPublisher>(nh_, parameters_.command_out_topic, active_joint_names);
    }
    else if (parameters_.publish_joint_velocities)
    {
      outgoing_cmd_interface_ =
          std::make_unique<Float64VelocityPublisher>(nh_, parameters_.command_out_topic, active_joint_names);
    }
  }
  else if (parameters_.command_out_type == "geometry_msgs/PoseStamped")
  {
    outgoing_cmd_interface_ = std::make_unique<PosePublisher>(nh_, parameters_.command_out_topic, active_joint_names);
  }
  // Publish status
  status_pub_ = nh_.advertise<std_msgs::Int8>(parameters_.status_topic, ROS_QUEUE_SIZE);

  ROS_INFO("Complile time: %s %s", __DATE__, __TIME__);

  internal_joint_state_.name = getJointNames();
  internal_joint_state_.position.resize(getNumJoints());
  internal_joint_state_.velocity.resize(getNumJoints());

  // A matrix of all zeros is used to check whether matrices have been
  // initialized
  auto angular_callback =
      std::bind(&ServoCalcs::angularScaleReconfigureCallback, this, std::placeholders::_1, std::placeholders::_2);
  m_dyn_conf_angular_scale_server =
      std::make_shared<AngularScaleServer>(ros::NodeHandle(ros::this_node::getName() + "/angular_scale"));
  m_dyn_conf_angular_scale_server->setCallback(angular_callback);

  auto linear_callback =
      std::bind(&ServoCalcs::linearScaleReconfigureCallback, this, std::placeholders::_1, std::placeholders::_2);
  m_dyn_conf_linear_scale_server = std::make_shared<LinearScaleServer>(ros::NodeHandle("~/linear_scale"));
  m_dyn_conf_linear_scale_server->setCallback(linear_callback);

  if (!m_cmd_validator.init(current_state_, parameters_))
    tf_moveit_to_ee_frame_ = Eigen::Matrix3d::Zero();

  if (parameters_.check_collisions)
  {
    if (parameters_.collision_check_type == "threshold_distance")
    {
      collision_checker_ = std::make_unique<ThresholdDistanceCollisionCheck>(nh_, parameters_, planning_scene_monitor_);
    }
    else if (parameters_.collision_check_type == "bullet_collision")
    {
      collision_checker_ = std::make_unique<BulletCollisionCheck>(nh_, parameters_, planning_scene_monitor_);
    }

    else if (!getRobotAcceleration())
    {
      ROS_WARN_NAMED(LOGNAME, "An acceleration limit is not defined for some joints. Minimum stop "
                              "distance not be used for collision checking. Threshold distance "
                              "will used instead");
      collision_checker_ = std::make_unique<ThresholdDistanceCollisionCheck>(nh_, parameters_, planning_scene_monitor_);
    }
    else
    {
      collision_checker_ = std::make_unique<StopDistanceCollisionCheck>(nh_, parameters_, planning_scene_monitor_);
    }
  }
}

ServoCalcs::~ServoCalcs()
{
  stop();
}

void ServoCalcs::start()
{
  // Stop the thread if we are currently running
  stop();

  // We will update last_sent_command_ every time we start servo
  auto initial_joint_trajectory = moveit::util::make_shared_from_pool<trajectory_msgs::JointTrajectory>();

  // When a joint_trajectory_controller receives a new command, a stamp of 0
  // indicates "begin immediately" See
  // http://wiki.ros.org/joint_trajectory_controller#Trajectory_replacement
  initial_joint_trajectory->header.stamp = ros::Time(0);
  initial_joint_trajectory->header.frame_id = parameters_.planning_frame;
  initial_joint_trajectory->joint_names = internal_joint_state_.name;
  trajectory_msgs::JointTrajectoryPoint point;
  point.time_from_start = ros::Duration(parameters_.publish_period);

  updateJoints();
  point.positions = internal_joint_state_.position;
  if (parameters_.publish_joint_positions)
    planning_scene_monitor_->getStateMonitor()->getCurrentState()->copyJointGroupPositions(joint_model_group_,
                                                                                           point.positions);
  if (parameters_.publish_joint_velocities)
  {
    point.velocities.resize(getNumJoints(), 0);
  }

  initial_joint_trajectory->points.push_back(point);
  last_sent_command_ = initial_joint_trajectory;
  m_last_calculated = ros::Time::now();
  current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  tf_moveit_to_ee_frame_ = current_state_->getGlobalLinkTransform(parameters_.planning_frame).inverse() *
                           current_state_->getGlobalLinkTransform(parameters_.ee_frame_name);
  tf_moveit_to_robot_cmd_frame_ = current_state_->getGlobalLinkTransform(parameters_.planning_frame).inverse() *
                                  current_state_->getGlobalLinkTransform(parameters_.robot_link_command_frame);

  stop_requested_ = false;
  thread_ = std::thread([this] { mainCalcLoop(); });
  new_input_cmd_ = false;
}

void ServoCalcs::stop()
{
  // Request stop
  stop_requested_ = true;

  // Notify condition variable in case the thread is blocked on it
  {
    // scope so the mutex is unlocked after so the thread can continue
    // and therefore be joinable
    const std::lock_guard<std::mutex> lock(input_mutex_);
    new_input_cmd_ = false;
    input_cv_.notify_all();
  }

  // Join the thread
  if (thread_.joinable())
  {
    thread_.join();
  }
}

void ServoCalcs::mainCalcLoop()
{
  ros::Rate rate(1.0 / parameters_.publish_period);

  while (ros::ok() && !stop_requested_)
  {
    // lock the input state mutex
    std::unique_lock<std::mutex> input_lock(input_mutex_);

    // low latency mode -- begin calculations as soon as a new command is
    // received.
    if (parameters_.low_latency_mode)
    {
      input_cv_.wait(input_lock, [this] { return (new_input_cmd_ || stop_requested_); });
    }

    // reset new_input_cmd_ flag
    new_input_cmd_ = false;

    // run servo calcs
    const auto start_time = ros::Time::now();
    calculateSingleIteration();
    auto cur_joint_velocity = (delta_theta_ / parameters_.publish_period).transpose().eval() * KDL::rad2deg;
    auto prev_joint_velocity = prev_joint_velocity_.transpose().eval() * KDL::rad2deg;
    auto joint_names = getJointNames();
    ROS_DEBUG_STREAM_NAMED("servo_calcs_variable",
                           "\n************BEGIN***************"
                               << std::boolalpha << std::fixed << std::setprecision(5)
                               << PRINT_VAR(have_nonzero_command_) << PRINT_VAR(have_nonzero_joint_command_)
                               << PRINT_VAR(have_nonzero_twist_stamped_) << PRINT_VAR(wait_for_servo_commands_)
                               << PRINT_VAR(updated_filters_) << PRINT_VAR(zero_velocity_count_)
                               << PRINT_VAR(twist_command_is_stale_) << PRINT_VAR(joint_command_is_stale_)
                               << PRINT_VAR(cur_joint_velocity) << PRINT_VAR(prev_joint_velocity)
                               << PRINT_VAR(joint_names) << "\n*************END**************");
    const auto run_duration = ros::Time::now() - start_time;

    // Log warning when the run duration was longer than the period
    if (run_duration.toSec() > parameters_.publish_period)
    {
      ROS_WARN_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME,
                                     "run_duration: " << run_duration.toSec() << " (" << parameters_.publish_period
                                                      << ")");
    }

    // normal mode, unlock input mutex and wait for the period of the loop
    if (!parameters_.low_latency_mode)
    {
      input_lock.unlock();
      rate.sleep();
    }
  }
}

bool ServoCalcs::convertDeltasToOutgoingCmd(trajectory_msgs::JointTrajectory& joint_trajectory)
{
  internal_joint_state_ = original_joint_state_;
  if (!addJointIncrements(internal_joint_state_, delta_theta_))
    return false;

  lowPassFilterPositions(internal_joint_state_);

  // Calculate joint velocities here so that positions are filtered and SRDF
  // bounds still get checked
  calculateJointVelocities(internal_joint_state_, delta_theta_);

  composeJointTrajMessage(internal_joint_state_, joint_trajectory);

  if (!enforcePositionLimits())
  {
    suddenHalt(joint_trajectory);
    status_ = StatusCode::JOINT_BOUND;
  }

  // done with calculations
  if (parameters_.use_gazebo)
  {
    insertRedundantPointsIntoTrajectory(joint_trajectory, gazebo_redundant_message_count_);
  }

  return true;
}

// Spam several redundant points into the trajectory. The first few may be
// skipped if the time stamp is in the past when it reaches the client. Needed
// for gazebo simulation.
void ServoCalcs::insertRedundantPointsIntoTrajectory(trajectory_msgs::JointTrajectory& joint_trajectory, int count) const
{
  joint_trajectory.points.resize(count);
  auto point = joint_trajectory.points[0];
  // Start from 2nd point (i = 1) because we already have the first point.
  // The timestamps are shifted up one period since first point is at 1 *
  // publish_period, not 0.
  for (int i = 1; i < count; ++i)
  {
    point.time_from_start = ros::Duration((i + 1) * parameters_.publish_period);
    joint_trajectory.points[i] = point;
  }
}

void ServoCalcs::lowPassFilterPositions(sensor_msgs::JointState& joint_state)
{
  std::size_t i = 0;
  std::for_each(joint_state.name.begin(), joint_state.name.end(), [this, &i, &joint_state](const std::string& name) {
    joint_state.position[i] = joint_model_map[name].position_filter_ptr_->filter(joint_state.position[i]);
    i++;
  });

  updated_filters_ = true;
}

void ServoCalcs::resetLowPassFilters(const sensor_msgs::JointState& joint_state)
{
  std::size_t i = 0;
  std::for_each(joint_state.name.begin(), joint_state.name.end(), [this, &i, &joint_state](const std::string& name) {
    joint_model_map[name].position_filter_ptr_->reset(joint_state.position[i]);
    i++;
  });
  updated_filters_ = true;
}
void ServoCalcs::calculateJointVelocities(sensor_msgs::JointState& joint_state, const Eigen::ArrayXd& delta_theta)
{
  for (int i = 0; i < delta_theta.size(); ++i)
  {
    joint_state.velocity[i] = delta_theta[i] / parameters_.publish_period;
  }
}

void ServoCalcs::composeJointTrajMessage(const sensor_msgs::JointState& joint_state,
                                         trajectory_msgs::JointTrajectory& joint_trajectory) const
{
  // When a joint_trajectory_controller receives a new command, a stamp of 0
  // indicates "begin immediately" See
  // http://wiki.ros.org/joint_trajectory_controller#Trajectory_replacement
  joint_trajectory.header.stamp = ros::Time(0);
  joint_trajectory.header.frame_id = parameters_.planning_frame;
  joint_trajectory.joint_names = joint_state.name;

  trajectory_msgs::JointTrajectoryPoint point;
  point.time_from_start = ros::Duration(parameters_.publish_period);
  point.positions = joint_state.position;
  point.velocities = joint_state.velocity;

  point.accelerations.resize(point.velocities.size());
  joint_trajectory.points.push_back(point);
}

// Apply velocity scaling for proximity of collisions and singularities.
void ServoCalcs::applyVelocityScaling(Eigen::ArrayXd& delta_theta, double singularity_scale)
{
  auto future_state = original_joint_state_;
  if (!addJointIncrements(future_state, delta_theta))
    delta_theta.setZero();

  double collision_scale = collision_checker_ ? collision_checker_->getScaleCoef(original_joint_state_, future_state) : 1.0; //FIXME

  if (collision_scale > 0 && collision_scale < 1)
  {
    status_ = StatusCode::DECELERATE_FOR_COLLISION;
    ROS_WARN_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME, SERVO_STATUS_CODE_MAP.at(status_));
  }
  //  else if (collision_scale == 0) //FIXME
  //  {//FIXME
  //    status_ = StatusCode::HALT_FOR_COLLISION; //FIXME
  //  }//FIXME

  delta_theta = collision_scale * singularity_scale * delta_theta;

  if (status_ == StatusCode::HALT_FOR_COLLISION)
  {
    ROS_WARN_STREAM_THROTTLE_NAMED(3, LOGNAME, "Halting for collision!");
    delta_theta_.setZero();
  }
}

// Possibly calculate a velocity scaling factor, due to proximity of singularity
// and direction of motion
double ServoCalcs::velocityScalingFactorForSingularity(const Eigen::VectorXd& commanded_velocity,
                                                       const Eigen::JacobiSVD<Eigen::MatrixXd>& svd,
                                                       const Eigen::MatrixXd& pseudo_inverse)
{
  double velocity_scale = 1;
  std::size_t num_dimensions = commanded_velocity.size();

  // Find the direction away from nearest singularity.
  // The last column of U from the SVD of the Jacobian points directly toward or
  // away from the singularity. The sign can flip at any time, so we have to do
  // some extra checking. Look ahead to see if the Jacobian's condition will
  // decrease.
  Eigen::VectorXd vector_toward_singularity = svd.matrixU().col(svd.matrixU().cols() - 1);

  double ini_condition = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);

  // This singular vector tends to flip direction unpredictably. See R. Bro,
  // "Resolving the Sign Ambiguity in the Singular Value Decomposition".
  // Look ahead to see if the Jacobian's condition will decrease in this
  // direction. Start with a scaled version of the singular vector
  Eigen::VectorXd delta_x(num_dimensions);
  double scale = 100;
  delta_x = vector_toward_singularity / scale;

  // Calculate a small change in joints
  Eigen::VectorXd new_theta;
  current_state_->copyJointGroupPositions(joint_model_group_, new_theta);
  new_theta += pseudo_inverse * delta_x;
  current_state_->setJointGroupPositions(joint_model_group_, new_theta);
  Eigen::MatrixXd new_jacobian = current_state_->getJacobian(joint_model_group_);

  Eigen::JacobiSVD<Eigen::MatrixXd> new_svd(new_jacobian);
  double new_condition = new_svd.singularValues()(0) / new_svd.singularValues()(new_svd.singularValues().size() - 1);
  // If new_condition < ini_condition, the singular vector does point towards a
  // singularity. Otherwise, flip its direction.
  if (ini_condition >= new_condition)
  {
    vector_toward_singularity *= -1;
  }

  // If this dot product is positive, we're moving toward singularity ==>
  // decelerate
  double dot = vector_toward_singularity.dot(commanded_velocity);
  if (dot > 0)
  {
    // Ramp velocity down linearly when the Jacobian condition is between
    // lower_singularity_threshold and hard_stop_singularity_threshold, and
    // we're moving towards the singularity
    if ((ini_condition > parameters_.lower_singularity_threshold) &&
        (ini_condition < parameters_.hard_stop_singularity_threshold))
    {
      velocity_scale = 1. - (ini_condition - parameters_.lower_singularity_threshold) /
                                (parameters_.hard_stop_singularity_threshold - parameters_.lower_singularity_threshold);
      status_ = StatusCode::DECELERATE_FOR_SINGULARITY;
      ROS_WARN_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME, SERVO_STATUS_CODE_MAP.at(status_));
    }

    // Very close to singularity, so halt.
    else if (ini_condition > parameters_.hard_stop_singularity_threshold)
    {
      velocity_scale = 0;
      status_ = StatusCode::HALT_FOR_SINGULARITY;
      ROS_WARN_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME, SERVO_STATUS_CODE_MAP.at(status_));
    }
  }

  return velocity_scale;
}

double ServoCalcs::enforceVelLimits(Eigen::ArrayXd& delta_theta)
{
  // Convert to joint angle velocities for checking and applying joint specific
  // velocity limits.
  Eigen::ArrayXd velocity = delta_theta / parameters_.publish_period;
  std::size_t joint_delta_index{ 0 };
  double velocity_scaling_factor{ 1.0 };
  std::for_each(joint_model_map.begin(), joint_model_map.end(),
                [&velocity, &joint_delta_index,
                 &velocity_scaling_factor](const decltype(joint_model_map)::value_type& pair) {
                  const auto& bound = pair.second.model_ptr->getVariableBounds();
                  if (bound[0].velocity_bounded_ && velocity(joint_delta_index) != 0.0)
                  {
                    const double unbounded_velocity = velocity(joint_delta_index);
                    // Clamp each joint velocity to a joint specific [min_velocity,
                    // max_velocity] range.
                    const auto bounded_velocity =
                        std::min(std::max(unbounded_velocity, bound[0].min_velocity_), bound[0].max_velocity_);
                    velocity_scaling_factor = std::min(velocity_scaling_factor, bounded_velocity / unbounded_velocity);
                  }
                  joint_delta_index++;
                });

  // Convert back to joint angle increments.
  delta_theta = velocity_scaling_factor * velocity * parameters_.publish_period;
  return velocity_scaling_factor;
}
tl::expected<Eigen::ArrayXd, std::string> ServoCalcs::getRobotAcceleration()
{
  Eigen::ArrayXd max_ddq(joint_model_map.size());
  auto index = 0;
  for (const auto& joint : joint_model_map)
  {
    auto joint_name = joint.first;
    auto joint_model = joint.second.model_ptr;
    auto joint_bound = joint_model->getVariableBounds();
    if (!joint_bound.empty() && joint_bound[0].acceleration_bounded_)
    {
      max_ddq(index++) = std::max(std::fabs(joint_bound[0].min_acceleration_), joint_bound[0].max_acceleration_);
    }
    else
    {
      return tl::make_unexpected('\"' + joint_name + "\" doesnt have acceleration limit");
    }
  }
  ROS_DEBUG_STREAM_NAMED("max_ddq", "Max ddq: " << max_ddq.transpose() * KDL::rad2deg);
  return max_ddq;
}
double ServoCalcs::enforceAccLimits(Eigen::ArrayXd& delta_theta)
{
  auto max_ddq = getRobotAcceleration();
  if (!max_ddq)
  {
    ROS_WARN_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME,
                                   ros::this_node::getName()
                                       << "Not apply acceleration limit reason: " << max_ddq.error());
    return 1.0;
  }

  Eigen::ArrayXd current_command_velocity = delta_theta / parameters_.publish_period;

  //  Eigen::ArrayXd current_velocity =
  //      Eigen::Map<Eigen::ArrayXd>(original_joint_state_.velocity.data(), original_joint_state_.velocity.size());

  //  Eigen::ArrayXd ddq_current =
  //      (current_velocity - prev_joint_velocity_) / (ros::Time::now() - m_last_calculated).toSec();
  //  Eigen::ArrayXd ddq_current = (current_command_velocity - current_velocity) / parameters_.publish_period;

  Eigen::ArrayXd ddq_current = (current_command_velocity - prev_joint_velocity_) / parameters_.publish_period;
  double max_scale = (ddq_current.abs() / (max_ddq.value())).maxCoeff();
  if (max_scale > 1.0)
  {
    auto scaled_ddq = (ddq_current / max_scale) * parameters_.publish_period;
    //    Eigen::ArrayXd new_dq = current_velocity + scaled_ddq;
    Eigen::ArrayXd new_dq = prev_joint_velocity_ + scaled_ddq;
    delta_theta = new_dq * parameters_.publish_period;
  }
  return max_scale;
}

bool ServoCalcs::enforcePositionLimits()
{
  bool halting = false;
  for (auto& joint : joint_model_map)
  {
    const auto joint_model_ptr = joint.second.model_ptr;
    auto it = std::find(original_joint_state_.name.begin(), original_joint_state_.name.end(), joint.first);
    if (it == original_joint_state_.name.end())
    {
      ROS_WARN_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME,
                                     ros::this_node::getName() << " " << __PRETTY_FUNCTION__ << " " << joint.first
                                                               << " not found in original_joint_state_");
      break;
    }
    auto index = static_cast<decltype(original_joint_state_.position)::size_type>(
        std::distance(original_joint_state_.name.begin(), it));
    double joint_angle = original_joint_state_.position.at(index);
    if (!current_state_->satisfiesPositionBounds(joint_model_ptr, -parameters_.joint_limit_margin))
    {
      const std::vector<moveit_msgs::JointLimits> limits = joint_model_ptr->getVariableBoundsMsg();

      // Joint limits are not defined for some joints. Skip them.
      if (!limits.empty())
      {
        if ((current_state_->getJointVelocities(joint_model_ptr)[0] < 0 &&
             (joint_angle < (limits[0].min_position + parameters_.joint_limit_margin))) ||
            (current_state_->getJointVelocities(joint_model_ptr)[0] > 0 &&
             (joint_angle > (limits[0].max_position - parameters_.joint_limit_margin))))
        {
          ROS_WARN_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME,
                                         ros::this_node::getName() << " " << joint.first
                                                                   << " close to a "
                                                                      " position limit. Halting.");
          halting = true;
        }
      }
    }
  }
  return !halting;
}

// Suddenly halt for a joint limit or other critical issue.
// Is handled differently for position vs. velocity control.
bool ServoCalcs::suddenHalt(trajectory_msgs::JointTrajectory& joint_trajectory, bool immidiate_stop)
{
  bool return_non_zero = false;
  if (immidiate_stop)
  {
    // Prepare the joint trajectory message to stop the robot
    joint_trajectory.joint_names = getJointNames();
    joint_trajectory.points.clear();
    joint_trajectory.points.emplace_back();
    trajectory_msgs::JointTrajectoryPoint& point = joint_trajectory.points.front();

    // When sending out trajectory_msgs/JointTrajectory type messages, the
    // "trajectory" is just a single point. That point cannot have the same
    // timestamp as the start of trajectory execution since that would mean the
    // arm has to reach the first trajectory point the moment execution begins. To
    // prevent errors about points being 0 seconds in the past, the smallest
    // supported timestep is added as time from start to the trajectory point.
    point.time_from_start.fromNSec(1);

    point.positions.resize(getNumJoints());
    point.velocities.resize(getNumJoints());

    // Assert the following loop is safe to execute
    assert(original_joint_state_.position.size() >= getNumJoints());

    // Set the positions and velocities vectors
    for (std::size_t i = 0; i < getNumJoints(); ++i)
    {
      // For position-controlled robots, can reset the joints to a known, good
      // state
      point.positions[i] = original_joint_state_.position[i];
      point.velocities[i] = 0;
      prev_joint_velocity_(i) = 0;
    }
  }
  else
  {
    delta_theta_.setZero(getNumJoints());
    enforceAccLimits(delta_theta_);
    if (!delta_theta_.isZero())
    {
      return_non_zero = true;
      prev_joint_velocity_ = delta_theta_ / parameters_.publish_period;
    }
    convertDeltasToOutgoingCmd(joint_trajectory);
  }
  return return_non_zero;
}

//// Parse the incoming joint msg for the joints of our MoveGroup
void ServoCalcs::updateJoints()
{
  // Get the latest joint group positions
  current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  if (internal_joint_state_.name.size() != getNumJoints())
  {
    ROS_INFO("Update internal_joint_state_ map");
    auto new_size = getNumJoints();
    internal_joint_state_.name = getJointNames();
    internal_joint_state_.position.resize(new_size);
    internal_joint_state_.velocity.resize(new_size);
  }
  decltype(joint_model_map)::size_type index = 0;
  std::for_each(joint_model_map.begin(), joint_model_map.end(),
                [this, &index](decltype(joint_model_map)::const_reference& pair) {
                  internal_joint_state_.position.at(index) = current_state_->getVariablePosition(pair.first);
                  internal_joint_state_.velocity.at(index++) = current_state_->getVariableVelocity(pair.first);
                });
  // Cache the original joints in case they need to be reset
  original_joint_state_ = internal_joint_state_;

  if (collision_checker_)
  {
    if (collision_checker_->getType() == CollisionCheckType::K_STOP_DISTANCE)
    {
      // Calculate worst case joint stop time, for collision checking
      double accel_limit = 0;
      double joint_velocity = 0;
      double worst_case_stop_time = 0;
      for (size_t jt_state_idx = 0; jt_state_idx < internal_joint_state_.velocity.size(); ++jt_state_idx)
      {
        std::string joint_name = internal_joint_state_.name[jt_state_idx];
        moveit::core::JointModel::Bounds kinematic_bounds = joint_model_map[joint_name].model_ptr->getVariableBounds();
        // Get acceleration limit for this joint
        // Some joints do not have acceleration limits
        if (kinematic_bounds[0].acceleration_bounded_)
        {
          // Be conservative when calculating overall acceleration limit from
          // min and max limits
          accel_limit =
              std::min(fabs(kinematic_bounds[0].min_acceleration_), fabs(kinematic_bounds[0].max_acceleration_));
        }
        else
        {
          ROS_WARN_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME,
                                         "An acceleration limit is not defined for this joint; minimum "
                                         "stop "
                                         "distance "
                                         "should not be used for collision checking");
        }
        // Get the current joint velocity
        joint_velocity = internal_joint_state_.velocity[jt_state_idx];
        // Calculate worst case stop time
        worst_case_stop_time = std::max(worst_case_stop_time, fabs(joint_velocity / accel_limit));
      }
      // publish message
      {
        auto msg = moveit::util::make_shared_from_pool<std_msgs::Float64>();
        msg->data = worst_case_stop_time;
        worst_case_stop_time_pub_.publish(msg);
      }
      //      collision_checker_->setWorstCaseStop(worst_case_stop_time);
    }
  }
}
// Scale the incoming servo command
Eigen::VectorXd ServoCalcs::scaleCartesianCommand(const geometry_msgs::TwistStamped& command) const
{
  Eigen::VectorXd result(6);
  result << command.twist.linear.x, command.twist.linear.y, command.twist.linear.z, command.twist.angular.x,
      command.twist.angular.y, command.twist.angular.z;
  // Получаем линейную и угловую компоненту скорости
  double linear_norm = result.head(3).norm();
  double angular_norm = result.tail(3).norm();
  if (parameters_.command_in_type == CommandInType::speed_units)
  {
    // Если какой-то из компонент превосходит асболютному значению максимальную
    // угловую или линейную скорость, то ограничиваем эту компоненту скорости,
    // обрезая вектор
    if (linear_norm > parameters_.linear_scale)
    {
      result.head(3).array() /= (linear_norm / parameters_.linear_scale);
    }
    if (angular_norm > parameters_.rotational_scale)
    {
      result.tail(3).array() /= (angular_norm / parameters_.rotational_scale);
    }
  }
  if (parameters_.command_in_type == CommandInType::unitless)
  {
    // Если с джойстика приходит только желаемое направление, обрезаем до
    // единичной длины вектора при привышении
    if (linear_norm > 1.0)
      result.head(3).array() /= linear_norm;
    if (angular_norm > 1.0)
      result.tail(3).array() /= angular_norm;
    result.head(3).array() *= parameters_.linear_scale;
    result.tail(3).array() *= parameters_.rotational_scale;
  }
  result.head(3).array() *= parameters_.publish_period * linear_scale.load();
  result.tail(3).array() *= parameters_.publish_period * angular_scale.load();
  return result;
}

// Add the deltas to each joint
bool ServoCalcs::addJointIncrements(sensor_msgs::JointState& output, const Eigen::VectorXd& increments) const
{
  if (increments.size() != output.position.size())
  {
    ROS_ERROR_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME,
                                    ros::this_node::getName() << " Lengths of output and "
                                                                 "increments do not match.");
    return false;
  }
  std::size_t increment_index = 0;
  for (auto& name : output.name)
  {
    auto it = joint_model_map.find(name);
    if (it == joint_model_map.end())
    {
      ROS_WARN_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME, "Ignoring joint " << name);
      return false;
    }
    auto c = std::distance(joint_model_map.begin(), it);
    output.position[c] += increments[increment_index++];
  }
  return true;
}

void ServoCalcs::removeDimension(Eigen::MatrixXd& jacobian, Eigen::VectorXd& delta_x, unsigned int row_to_remove)
{
  unsigned int num_rows = jacobian.rows() - 1;
  unsigned int num_cols = jacobian.cols();

  if (row_to_remove < num_rows)
  {
    jacobian.block(row_to_remove, 0, num_rows - row_to_remove, num_cols) =
        jacobian.block(row_to_remove + 1, 0, num_rows - row_to_remove, num_cols);
    delta_x.segment(row_to_remove, num_rows - row_to_remove) =
        delta_x.segment(row_to_remove + 1, num_rows - row_to_remove);
  }
  jacobian.conservativeResize(num_rows, num_cols);
  delta_x.conservativeResize(num_rows);
}

bool ServoCalcs::getCommandFrameTransform(Eigen::Isometry3d& transform)
{
  const std::lock_guard<std::mutex> lock(input_mutex_);
  transform = tf_moveit_to_robot_cmd_frame_;

  // All zeros means the transform wasn't initialized, so return false
  return !transform.matrix().isZero(0);
}

bool ServoCalcs::getCommandFrameTransform(geometry_msgs::TransformStamped& transform)
{
  const std::lock_guard<std::mutex> lock(input_mutex_);
  // All zeros means the transform wasn't initialized, so return false
  if (tf_moveit_to_robot_cmd_frame_.matrix().isZero(0))
  {
    return false;
  }

  transform = convertIsometryToTransform(tf_moveit_to_robot_cmd_frame_, parameters_.planning_frame,
                                         parameters_.robot_link_command_frame);
  return true;
}

bool ServoCalcs::getEEFrameTransform(Eigen::Isometry3d& transform)
{
  const std::lock_guard<std::mutex> lock(input_mutex_);
  transform = tf_moveit_to_ee_frame_;

  // All zeros means the transform wasn't initialized, so return false
  return !transform.matrix().isZero(0);
}

bool ServoCalcs::getEEFrameTransform(geometry_msgs::TransformStamped& transform)
{
  const std::lock_guard<std::mutex> lock(input_mutex_);
  // All zeros means the transform wasn't initialized, so return false
  if (tf_moveit_to_ee_frame_.matrix().isZero(0))
  {
    return false;
  }

  transform = convertIsometryToTransform(tf_moveit_to_ee_frame_, parameters_.planning_frame, parameters_.ee_frame_name);
  return true;
}

void ServoCalcs::twistStampedCB(const geometry_msgs::TwistStampedConstPtr& msg)
{
  const std::lock_guard<std::mutex> lock(input_mutex_);
  latest_twist_stamped_ = msg;
  latest_nonzero_twist_stamped_ = isNonZero(*latest_twist_stamped_);

  if (msg->header.stamp != ros::Time(0.))
    latest_twist_command_stamp_ = msg->header.stamp;

  // notify that we have a new input
  new_input_cmd_ = true;
  input_cv_.notify_all();
}

void ServoCalcs::collisionVelocityScaleCB(const std_msgs::Float64ConstPtr& msg)
{
  collision_velocity_scale_ = msg->data;
}

bool ServoCalcs::changeDriftDimensions(moveit_msgs::ChangeDriftDimensions::Request& req,
                                       moveit_msgs::ChangeDriftDimensions::Response& res)
{
  drift_dimensions_[0] = req.drift_x_translation;
  drift_dimensions_[1] = req.drift_y_translation;
  drift_dimensions_[2] = req.drift_z_translation;
  drift_dimensions_[3] = req.drift_x_rotation;
  drift_dimensions_[4] = req.drift_y_rotation;
  drift_dimensions_[5] = req.drift_z_rotation;

  res.success = true;
  return true;
}

bool ServoCalcs::changeControlDimensions(moveit_msgs::ChangeControlDimensions::Request& req,
                                         moveit_msgs::ChangeControlDimensions::Response& res)
{
  control_dimensions_[0] = req.control_x_translation;
  control_dimensions_[1] = req.control_y_translation;
  control_dimensions_[2] = req.control_z_translation;
  control_dimensions_[3] = req.control_x_rotation;
  control_dimensions_[4] = req.control_y_rotation;
  control_dimensions_[5] = req.control_z_rotation;

  res.success = true;
  return true;
}

bool ServoCalcs::resetServoStatus(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
{
  status_ = StatusCode::NO_WARNING;
  return true;
}
uint ServoCalcs::getNumJoints() const
{
  return static_cast<uint>(joint_model_map.size());
}

std::vector<std::string> ServoCalcs::getJointNames() const
{
  std::vector<std::string> result(getNumJoints());
  std::size_t i = 0;
  std::for_each(joint_model_map.begin(), joint_model_map.end(), [&result, &i](const auto& pair) {
    result[i] = pair.first;
    i++;
  });
  return result;
}

void ServoCalcs::setPaused(bool paused)
{
  paused_ = paused;
}

void ServoCalcs::changeRobotLinkCommandFrame(const std::string& new_command_frame)
{
  parameters_.robot_link_command_frame = new_command_frame;
}

}  // namespace moveit_servo
