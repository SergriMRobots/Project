#include "moveit_servo/command_publishers/pose_publisher.h"
#include "moveit_servo/servo_parameters.h"
#include <string>
namespace moveit_servo
{
PosePublisher::PosePublisher(ros::NodeHandle& nh_, const std::string& output_topic,
                             const std::vector<std::string>& joints_order)
  : joints_order(joints_order)
{
  std::string base_link;
  std::string ee_link;
  nh_.getParam("pose_stamped/robot_base_link", base_link);
  nh_.getParam("pose_stamped/end_effector_link", ee_link);

  outgoing_cmd_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(output_topic, moveit_servo::ROS_QUEUE_SIZE);

  KDL::Tree tree;
  kdl_parser::treeFromParam("/robot_description", tree);
  if (!tree.getChain(base_link, ee_link, chain))
  {
    ROS_ERROR("Error in getting chain from '%s' to '%s'", base_link.c_str(), ee_link.c_str());
    throw std::runtime_error("Error in getting chain from world to ee_link");
  }
  msg.header.frame_id = base_link;
  fk_solver = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain);
}

void PosePublisher::publish(trajectory_msgs::JointTrajectory& traj)
{
  auto index = getIndecesArray(traj, joints_order);
  if (!index)
  {
    ROS_ERROR("Indeces array return this error \'%s\' Skip publishing", index.error().c_str());
    return;
  }
  KDL::JntArray jnt_array(index.value().size());
  for (std::size_t i = 0; i < index->size(); i++)
  {
    jnt_array(index.value()[i]) = traj.points[0].positions[i];
  }

  KDL::Frame frame;
  if (fk_solver->JntToCart(jnt_array, frame) == KDL::SolverI::E_NOERROR)
  {
    msg.header.stamp = ros::Time::now();
    tf::poseKDLToMsg(frame, msg.pose);
    outgoing_cmd_pub_.publish(msg);
  }
}
}  // namespace moveit_servo
