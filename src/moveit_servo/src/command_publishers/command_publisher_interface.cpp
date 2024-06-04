#include "moveit_servo/command_publishers/command_publisher_interface.h"
#include <algorithm>
#include <cstddef>
#include <iterator>
#include <ostream>
#include "moveit_servo/3rdparty_header/expected.hpp"
namespace moveit_servo
{
tl::expected<CommandPublisherInterface::IndecesArray, CommandPublisherInterface::ErrorMsg>
CommandPublisherInterface::getIndecesArray(const trajectory_msgs::JointTrajectory& traj,
                                           const std::vector<std::string>& joints_order)
{
  if (traj.points.empty())
  {
    return tl::make_unexpected("Input trajectory is empty");
  }
  if (traj.joint_names.size() < joints_order.size())
  {
    return tl::make_unexpected("Input trajectory have less joint_names then joints_order");
  }
  CommandPublisherInterface::IndecesArray array(joints_order.size());
  std::size_t counter = 0;
  for (const auto& joint : joints_order)
  {
    auto traj_name_begin = std::begin(traj.joint_names);
    auto traj_name_end = std::end(traj.joint_names);
    auto it = std::find(traj_name_begin, traj_name_end, joint);
    if (it == traj_name_end)
    {
      return tl::make_unexpected("Not found \'" + joint + "\' in trajectory joint names");
    }
    auto joint_index = std::distance(traj_name_begin, it);
    array[counter++] = joint_index;
  }
  return array;
}  // namespace moveit_servo

}  // namespace moveit_servo
