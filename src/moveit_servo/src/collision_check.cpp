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

/*      Title     : collision_check.cpp
 *      Project   : moveit_servo
 *      Created   : 1/11/2019
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
 */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_servo/collision_check.h>
#include <moveit_servo/make_shared_from_pool.h>
#include <std_msgs/Float64.h>

namespace moveit_servo
{
using namespace collision_detection;

std::ostream& operator<<(std::ostream& os, const Eigen::Vector3d& vec)
{
  os << "X: " << vec.x() << " Y: " << vec.y() << " Z: " << vec.z();
  return os;
}
std::ostream& operator<<(std::ostream& os, const DistanceMap& result_map)
{
  for (auto& el : result_map)
  {
    os << "Key pair first: " << std::quoted(el.first.first) << " second: " << std::quoted(el.first.second) << "\n";
    for (auto i = 0; i < el.second.size(); i++)
    {
      os << "Distance data #" << i << '\n';
      os << el.second[i];
    }
  }
  return os;
}

std::ostream& operator<<(std::ostream& os, const DistanceResultsData& result_data)
{
  auto body_type_str = [](const BodyType& bt) -> std::string {
    switch (bt)
    {
      case BodyType::ROBOT_ATTACHED:
        return "ROBOT_ATTACHED";
      case BodyType::ROBOT_LINK:
        return "ROBOT_LINK";
      case BodyType::WORLD_OBJECT:
        return "WORLD_OBJECT";
      default:
        return "UNKNOWN_OBJECT";
    }
  };

  os << "Link#1: " << std::quoted(result_data.link_names[0]) << " (Type: " << body_type_str(result_data.body_types[0])
     << '[' << result_data.body_types[0] << "]) ";
  os << "Link#2: " << std::quoted(result_data.link_names[1]) << " (Type: " << body_type_str(result_data.body_types[1])
     << '[' << result_data.body_types[1] << "])\n";
  os << "Distance: " << result_data.distance * 1000 << " mm\n";
  //  os << "Nearest point#1: " << result_data.nearest_points[0] << ' ';
  //  os << "Nearest point#2: " << result_data.nearest_points[1] << '\n';
  //  os << "Normal from 1 to 2: " << result_data.normal << '\n';
  return os;
}
std::ostream& operator<<(std::ostream& os, const CollisionCheck::CollisionCheckingResult& result)
{
  os << "Self min dist: " << result.self_minimum_distance.distance * 1000 << " mm between "
     << std::quoted(result.self_minimum_distance.link_names[0]) << " and "
     << std::quoted(result.self_minimum_distance.link_names[1]);
  if (result.use_scene)
    os << "Scene min dist: " << result.scene_minimum_distance.distance * 1000 << " mm between "
       << std::quoted(result.scene_minimum_distance.link_names[0]) << " and "
       << std::quoted(result.scene_minimum_distance.link_names[1]);
  return os;
}
// Constructor for the class that handles collision checking
CollisionCheck::CollisionCheck(ros::NodeHandle& nh, const moveit_servo::ServoParameters& parameters,
                               const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
  : nh_(nh)
  , parameters_(parameters)
  , planning_scene_monitor_(planning_scene_monitor)
  , state(*planning_scene_monitor_->getStateMonitor()->getCurrentState())
  , use_collision_check(parameters.check_collisions)
{
  // Init collision request
  collision_request_.group_name = parameters_.move_group_name;
  collision_request_.distance = true;  // enable distance-based collision checking
  collision_request_.contacts = true;  // Record the names of collision pairs
  acm_ = getLockedPlanningSceneRO()->getAllowedCollisionMatrix();
  collision_enable_service =
      nh_.advertiseService(parameters.collisions_switch_service, &CollisionCheck::collisionHandler, this);
  ROS_WARN("Collision cheker is \'%s\'", parameters.check_collisions ? "True" : "False");
  ROS_DEBUG_STREAM_NAMED("max_elpased", "Init");
  ROS_DEBUG_STREAM_NAMED("collision_check_scale", "Init");
  ROS_DEBUG_STREAM_NAMED("collision_check_elapsed", "Init");
  ROS_DEBUG_STREAM_NAMED("current_collision_distance", "Init");
  ROS_DEBUG_STREAM_NAMED("future_collision_distance", "Init");

  ROS_DEBUG_STREAM_NAMED("world_minimal_distance", "Init");
  ROS_DEBUG_STREAM_NAMED("world_distance_map", "Init");
  ROS_DEBUG_STREAM_NAMED("self_minimal_distance", "Init");
  ROS_DEBUG_STREAM_NAMED("self_distance_map", "Init");
}

CollisionCheck::~CollisionCheck()
{
}

planning_scene_monitor::LockedPlanningSceneRO CollisionCheck::getLockedPlanningSceneRO() const
{
  return planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_);
}

CollisionCheck::CollisionCheckingResult CollisionCheck::computeCollisionResultForState(moveit::core::RobotState& state)
{
  state.updateCollisionBodyTransforms();
  // Do a thread-safe distance-based collision detection
  std::list<DistanceResult> results;

#if PARALLEL_COLLISION_CHECK == 1
  {  // Lock PlanningScene
    auto scene_ro = getLockedPlanningSceneRO();

    std::list<std::future<DistanceResult>> collision_checkers;

    auto collision_robot = scene_ro->getCollisionEnvUnpadded();
    if (scene_ro->getWorld()->size() != 0)
    {
      auto collision_world = scene_ro->getCollisionEnv();
      collision_checkers.emplace_back(std::async(std::launch::async, [&collision_robot, &collision_world, this,
                                                                      &state]() {
        DistanceRequest dist_req_;
        dist_req_.enableGroup(collision_robot->getRobotModel());
        dist_req_.type = collision_detection::DistanceRequestType::ALL;
        dist_req_.acm = &acm_;
        dist_req_.group_name = parameters_.move_group_name;
        DistanceResult dist_res_;
        collision_world->distanceRobot(dist_req_, dist_res_, state);
        ROS_DEBUG_STREAM_NAMED("world_minimal_distance", "\n---------------------\n"
                                                             << "World minimal distance: " << dist_res_.minimum_distance
                                                             << "---------------------");
        ROS_DEBUG_STREAM_NAMED("world_distance_map", "\n---------------------\n"
                                                         << "Distance map world:\n"
                                                         << dist_res_.distances << "---------------------");
        return dist_res_;
      }));
    }
    collision_checkers.emplace_back(std::async(std::launch::async, [&collision_robot, this, &state]() {
      DistanceRequest dist_req_;
      dist_req_.enableGroup(collision_robot->getRobotModel());
      dist_req_.type = collision_detection::DistanceRequestType::ALL;
      dist_req_.acm = &acm_;
      dist_req_.max_contacts_per_body = 10;
      dist_req_.group_name = parameters_.move_group_name;
      DistanceResult dist_res_;
      collision_robot->distanceSelf(dist_req_, dist_res_, state);
      ROS_DEBUG_STREAM_NAMED("self_minimal_distance", "\n---------------------\n"
                                                          << "Self minimal distance: " << dist_res_.minimum_distance
                                                          << "---------------------");
      ROS_DEBUG_STREAM_NAMED("self_distance_map", "\n---------------------\n"
                                                      << "self map world:\n"
                                                      << dist_res_.distances << "---------------------");
      return dist_res_;
    }));

    for (auto& checker : collision_checkers)
    {
      checker.wait();
      results.emplace_back(checker.get());
    }
  }  // Unlock PlanningScene

#else
  {
    auto scene_ro = getLockedPlanningSceneRO();
#if ROS_VERSION_MINOR == 14    //Для melodic
    auto collision_world = scene_ro->getCollisionWorld();
    collision_world->checkRobotCollision(collision_request_, collision_result_scene_, *scene_ro->getCollisionRobot(),
                                         state, acm_);
    auto collision_robot = scene_ro->getCollisionRobotUnpadded();
    collision_robot->checkSelfCollision(collision_request_, collision_result_self_, state, acm_);
#elif ROS_VERSION_MINOR == 15  //Для noetic
    auto collision_world = scene_ro->getCollisionEnv();
    collision_world->checkRobotCollision(collision_request_, collision_result_scene_, state, acm_);
    auto collision_robot = scene_ro->getCollisionEnvUnpadded();
    collision_robot->checkSelfCollision(collision_request_, collision_result_self_, state, acm_);
#endif
  }

#endif

  if (results.size() == 2)
  {
    return CollisionCheckingResult(results.back(), results.front());
  }
  else
  {
    return CollisionCheckingResult(results.front());
  }
}

double CollisionCheck::getScaleCoef(const sensor_msgs::JointState& now, const sensor_msgs::JointState& future)
{
  if (use_collision_check)
  {
    static ros::Duration max_elapsed;
    auto start = ros::Time::now();
    auto scale = calcIteration(now, future);
    ROS_DEBUG_STREAM_NAMED("collision_check_scale", "Calculated scale : " << scale);
    auto current = ros::Time::now() - start;
    ROS_DEBUG_STREAM_NAMED("collision_check_elapsed", "Elapsed time: " << current);
    max_elapsed = std::max(current, max_elapsed);
    ROS_DEBUG_STREAM_NAMED("max_elpased", "Max elapsed time: " << max_elapsed);

    return scale;
  }
  ROS_WARN_THROTTLE(1, "Collision checking disable");
  return 1.0;
}

bool CollisionCheck::collisionHandler(std_srvs::TriggerRequest&, std_srvs::TriggerResponse& resp)
{
  bool current_state = use_collision_check;
  use_collision_check.store(!current_state);
  resp.success = !current_state;
  resp.message = "Collision ";
  resp.message += (!current_state) ? "Enable" : "Disable";
  return true;
}

}  // namespace moveit_servo
