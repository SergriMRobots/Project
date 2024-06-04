#include "moveit_servo/kdl_based_extensions/command_validator.h"

#include <kdl/chainiksolvervel_pinv.hpp>
#include <moveit_servo/3rdparty_header/expected.hpp>
namespace moveit_servo
{
tl::expected<urdf::ModelInterface, std::string> getURDFInterface(const moveit::core::RobotStatePtr& robot_state_ptr)
{
  if (!robot_state_ptr)
  {
    return tl::make_unexpected("robot_state_ptr ptr is empty");
  }
  if (!robot_state_ptr->getRobotModel())
  {
    return tl::make_unexpected("robot_state_ptr->getRobotModel() ptr is empty");
  }
  if (!robot_state_ptr->getRobotModel()->getURDF())
  {
    return tl::make_unexpected("robot_state_ptr->getRobotModel()->getURDF() ptr is empty");
  }
  return *robot_state_ptr->getRobotModel()->getURDF();
}
KDL::JntArrayVel CommandValidator::toKDL(moveit::core::RobotStatePtr current_state_)
{
  KDL::JntArrayVel q_and_dq(m_manipulator.getNrOfJoints());
  current_state_->copyJointGroupPositions(m_joint_group_name, q_and_dq.q.data);
  current_state_->copyJointGroupVelocities(m_joint_group_name, q_and_dq.qdot.data);
  return q_and_dq;
}

KDL::Twist CommandValidator::toKDL(const Eigen::VectorXd& command_twist)
{
  KDL::Twist result;
  for (auto i = 0; i < 6; ++i)
  {
    result(i) = command_twist(i);
  }
  return result;
}

bool CommandValidator::init(const moveit::core::RobotStatePtr& robot_state_ptr, const ServoParameters& parameters)
{
  auto urdf_interface = getURDFInterface(robot_state_ptr);
  if (!urdf_interface)
  {
    ROS_ERROR("Command validator: %s", urdf_interface.error().c_str());
    return false;
  }
  m_publish_period = parameters.publish_period;
  m_joint_group_name = parameters.move_group_name;
  auto joint_model_group_ = robot_state_ptr->getJointModelGroup(m_joint_group_name);
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(urdf_interface.value(), tree))
  {
    ROS_ERROR("CommandValidator: treeFromUrdfModel func error");
    return false;
  }

  std::string root_name;
  std::string end_effector_name;
  std::tie(root_name, end_effector_name) = joint_model_group_->getConfig().chains_[0];
  if (!tree.getChain(root_name, end_effector_name, m_manipulator))
  {
    ROS_ERROR("CommandValidator: treeFromUrdfModel func error");
    return false;
  }
  ROS_WARN("Parsed chain from \'%s\' to \'%s\'", root_name.c_str(), end_effector_name.c_str());

  return true;
}

bool CommandValidator::commandValidateControl(moveit::core::RobotStatePtr current_state_,
                                              const Eigen::VectorXd& delta_x, Eigen::ArrayXd& delta_theta_)
{
  auto current_q_dq = toKDL(current_state_);

  KDL::ChainFkSolverVel_recursive fk_vel_solver(m_manipulator);
  KDL::FrameVel currentFrameVel;
  if (fk_vel_solver.JntToCart(current_q_dq, currentFrameVel) != KDL::SolverI::E_NOERROR)
  {
    return false;
  }
  auto current_twist = currentFrameVel.GetTwist();
  auto target_twist = toKDL(delta_x / m_publish_period);

  double currentLinSpeed = current_twist.vel.Norm();
  double targetLinSpeed = target_twist.vel.Norm();

  double currentRotSpeed = current_twist.rot.Norm();
  double targetRotSpeed = target_twist.rot.Norm();
  double current_speed_coef = 2.0;
  ROS_DEBUG_STREAM_NAMED("command_validator",
                         std::fixed << std::setprecision(5) << "Current q:\n"
                                    << current_q_dq.q.data.transpose() * KDL::rad2deg << '\n'
                                    << "Current dq:\n"
                                    << current_q_dq.qdot.data.transpose() * KDL::rad2deg << '\n'
                                    << "Current linear speed: " << currentLinSpeed * 1000.0 << " mm\\s\n"
                                    << "Command linear speed: " << targetLinSpeed * 1000.0 << " mm\\s\n"
                                    << "Current angular speed: " << currentRotSpeed * KDL::rad2deg << "deg\\s\n"
                                    << "Command angular speed: " << targetRotSpeed * KDL::rad2deg << " deg\\s\n");

  if (currentLinSpeed * 1000 > 5 && targetLinSpeed * 1000 > 5)
  {
    double angleVel = std::acos(KDL::dot(target_twist.vel, current_twist.vel) / (currentLinSpeed * targetLinSpeed));
    //    ROS_INFO("Lin angle: %f", angleVel * KDL::rad2deg);
    if (angleVel > 90 * KDL::deg2rad)
    {
      ROS_WARN("Lin angle more than 90. Delta theta sets in %f times less of "
               "current speed",
               current_speed_coef);
      KDL::ChainIkSolverVel_pinv speedSolver(m_manipulator, 0.00001, 1000);
      KDL::JntArray q_out(6);
      speedSolver.CartToJnt(current_q_dq.q,
                            KDL::Twist(current_twist.vel / current_speed_coef, current_twist.rot / current_speed_coef),
                            q_out);
      delta_theta_ = q_out.data * m_publish_period;
      return true;
    }
  }

  if (currentRotSpeed * KDL::rad2deg > 1 && targetRotSpeed * KDL::rad2deg > 1)
  {
    double angleRotVel = std::acos(KDL::dot(target_twist.rot, current_twist.rot) / (currentRotSpeed * targetRotSpeed));
    //    ROS_INFO("Rot angle: %f", angleRotVel * KDL::rad2deg);
    if (angleRotVel > 90 * KDL::deg2rad)
    {
      ROS_WARN("Rot angle more than 90. Delta theta sets in %f times less of "
               "current speed",
               current_speed_coef);
      KDL::ChainIkSolverVel_pinv speedSolver(m_manipulator, 0.00001, 1000);
      KDL::JntArray q_out(6);
      speedSolver.CartToJnt(current_q_dq.q,
                            KDL::Twist(current_twist.vel / current_speed_coef, current_twist.rot / current_speed_coef),
                            q_out);
      delta_theta_ = q_out.data * m_publish_period;
      return true;
    }
  }
  return true;
}

}  // namespace moveit_servo
