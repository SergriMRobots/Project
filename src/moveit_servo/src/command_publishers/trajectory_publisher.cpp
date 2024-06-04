#include "moveit_servo/command_publishers/trajectory_publisher.h"
#include "moveit_servo/servo_parameters.h"
namespace moveit_servo
{
TrajectoryPublisher::TrajectoryPublisher(ros::NodeHandle& nh_, const std::string& output_topic)
{
  outgoing_cmd_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(output_topic, moveit_servo::ROS_QUEUE_SIZE);
}

void TrajectoryPublisher::publish(trajectory_msgs::JointTrajectory& traj)
{
  traj.header.stamp = ros::Time(0);
  outgoing_cmd_pub_.publish(traj);
}
}  // namespace moveit_servo
