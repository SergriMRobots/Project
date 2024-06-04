#include "moveit_servo/command_publishers/float64_publisher.h"
#include "moveit_servo/servo_parameters.h"
#include <cstddef>
#include <utility>
#include <vector>
namespace moveit_servo
{
Float64Publisher::Float64Publisher(ros::NodeHandle& nh_, const std::string& output_topic,
                                   const std::vector<std::string>& joints_order)
  : joints_order(joints_order)
{
  outgoing_cmd_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(output_topic, moveit_servo::ROS_QUEUE_SIZE);
}
Float64VelocityPublisher::Float64VelocityPublisher(ros::NodeHandle& nh_, const std::string& output_topic,
                                                   const std::vector<std::string>& joints_order)
  : Float64Publisher(nh_, output_topic, joints_order)
{
}

Float64PositionPublisher::Float64PositionPublisher(ros::NodeHandle& nh_, const std::string& output_topic,
                                                   const std::vector<std::string>& joints_order)
  : Float64Publisher(nh_, output_topic, joints_order)
{
}

void Float64VelocityPublisher::publish(trajectory_msgs::JointTrajectory& traj)
{
  auto index = getIndecesArray(traj, joints_order);
  if (!index)
  {
    ROS_ERROR("Indeces array return this error \'%s\' Skip publishing", index.error().c_str());
    return;
  }
  msg.data.resize(traj.points[0].velocities.size());
  for (std::size_t i = 0; i < index->size(); i++)
  {
    msg.data[index.value()[i]] = traj.points[0].velocities[i];
  }
  ROS_DEBUG_STREAM_NAMED("vel_command", "\nInput:\n" << traj << "\nOutput:\n" << msg);
  outgoing_cmd_pub_.publish(msg);
}
void Float64PositionPublisher::publish(trajectory_msgs::JointTrajectory& traj)
{
  auto index = getIndecesArray(traj, joints_order);
  if (!index)
  {
    ROS_ERROR("Indeces array return this error \'%s\' Skip publishing", index.error().c_str());
    return;
  }
  msg.data.resize(traj.points[0].positions.size());
  for (std::size_t i = 0; i < index->size(); i++)
  {
    msg.data[index.value()[i]] = traj.points[0].positions[i];
  }
  ROS_DEBUG_STREAM_NAMED("pos_command", "\nInput:\n" << traj << "\nOutput:\n" << msg);
  outgoing_cmd_pub_.publish(msg);
}
}  // namespace moveit_servo
