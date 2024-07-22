/*******************************************************************************
 * Title     : collision_check.h
 * Project   : moveit_servo
 * Created   : 1/11/2019
 * Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
 *
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

#pragma once

#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_servo/low_pass_filter.h>
#include <moveit_servo/servo_parameters.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Trigger.h>

#include <atomic>
#include <future>

#include "extensions/value_converter.h"

namespace moveit_servo
{
/**
 * @brief
 *
 */
enum CollisionCheckType
{
  K_THRESHOLD_DISTANCE = 1,
  K_STOP_DISTANCE = 2,
  K_BULLET_DISTANCE = 3
};

/**
 * @brief Класс интерфейс для методов проверки столковнений
 * Суть всех методов на основе текущего положения и положения расчитанного через
 * такт отправки команды скоростей высчитать можно ли зашарашить все скорости на
 * полную возвращаемый scaleCoef ==1 или надо притормаживать scaleCoef<1 или
 * нельзя вообще никуда ехать scaleCoef==0
 *
 */
class CollisionCheck
{
public:
  /** \brief Constructor
   *  \param parameters: common settings of moveit_servo
   *  \param planning_scene_monitor: PSM should have scene monitor and state
   * monitor already started when passed into this class
   */
  CollisionCheck(ros::NodeHandle& nh, const moveit_servo::ServoParameters& parameters,
                 const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor);

  virtual ~CollisionCheck();

  double getScaleCoef(const sensor_msgs::JointState& now, const sensor_msgs::JointState& delta_theta);

  CollisionCheckType getType()
  {
    return collision_check_type_;
  }
ros::Publisher dummy_distance_publisher_; //FIXME
planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_; //FIXME
collision_detection::AllowedCollisionMatrix acm_;//FIXME
private:
  bool collisionHandler(std_srvs::TriggerRequest&, std_srvs::TriggerResponse& resp);
  /** \brief Run one iteration of collision checking */
  virtual double calcIteration(const sensor_msgs::JointState& now, const sensor_msgs::JointState& future) = 0;

  ros::NodeHandle nh_;

  // Parameters from yaml
  const ServoParameters& parameters_;

  // Pointer to the collision environment
  // planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // // Robot state and collision matrix from planning scene
  // collision_detection::AllowedCollisionMatrix acm_;

  // ROS
  ros::ServiceServer collision_enable_service;
  bool check_world;

protected:
  struct CollisionCheckingResult
  {
    collision_detection::DistanceMap self_distances;
    collision_detection::DistanceMap scene_distances;
    collision_detection::DistanceResultsData self_minimum_distance;
    collision_detection::DistanceResultsData scene_minimum_distance;
    double minimalCollistionDist()
    {
      if (use_scene)
      {
        return std::min(self_minimum_distance.distance, scene_minimum_distance.distance);
      }
      return self_minimum_distance.distance;
    }
    bool use_scene = false;

    bool collision_detected = false;
    CollisionCheckingResult(const collision_detection::DistanceResult& self_result)
      : self_distances{ std::move(self_result.distances) }
      , self_minimum_distance{ std::move(self_result.minimum_distance) }
      , collision_detected{ self_result.collision }
    {
    }
    CollisionCheckingResult(const collision_detection::DistanceResult& self_result,
                            const collision_detection::DistanceResult& scene_result)
      : CollisionCheckingResult(self_result)
    {
      scene_distances = std::move(scene_result.distances);
      scene_minimum_distance = std::move(scene_result.minimum_distance);
      collision_detected |= scene_result.collision;
    }
  };
  CollisionCheckType collision_check_type_;
  // Scale robot velocity according to collision proximity and user-defined
  // thresholds. I scaled exponentially (cubic power) so velocity drops off
  // quickly after the threshold. Proximity decreasing --> decelerate
  // collision request
  collision_detection::CollisionRequest collision_request_;
  moveit::core::RobotState state;
  std::atomic_bool use_collision_check;

  /** \brief Get a read-only copy of the planning scene */
  planning_scene_monitor::LockedPlanningSceneRO getLockedPlanningSceneRO() const;
  CollisionCheckingResult computeCollisionResultForState(moveit::core::RobotState& state);
  friend std::ostream& operator<<(std::ostream& os, const CollisionCheckingResult& result);
};
std::ostream& operator<<(std::ostream& os, const CollisionCheck::CollisionCheckingResult& result);
std::ostream& operator<<(std::ostream& os, const collision_detection::DistanceResultsData& result_data);
}  // namespace moveit_servo
