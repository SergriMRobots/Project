#ifndef COMMAND_PUBLISHER_INTERFACE_H
#define COMMAND_PUBLISHER_INTERFACE_H
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <cstddef>
#include <vector>
#include "moveit_servo/3rdparty_header/expected.hpp"
namespace moveit_servo
{
class CommandPublisherInterface
{
protected:
  using IndecesArray = std::vector<std::size_t>;
  using ErrorMsg = std::string;

  ros::Publisher outgoing_cmd_pub_;
  ///
  /// \brief getIndecesArray возвращает массив индексов шарниров в traj чтобы перераспределить их в порядке joints_order
  /// \param traj траектория сгенерированная сервосервером
  /// \param joints_order - порядок имен шарниров в котором надо заслать траекторию на выполнение
  /// \return возвращает expected объект, так как если не все шарниры из joints_order могут быть в traj, тогда в
  /// expected будет ошибка, в противном случае joints_order
  ///
  static tl::expected<IndecesArray, ErrorMsg> getIndecesArray(const trajectory_msgs::JointTrajectory& traj,
                                                              const std::vector<std::string>& joints_order);

public:
  virtual ~CommandPublisherInterface()
  {
  }

  virtual void publish(trajectory_msgs::JointTrajectory& traj) = 0;
};
}  // namespace moveit_servo
#endif  // COMMAND_PUBLISHER_INTERFACE_H
