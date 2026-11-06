// Copyright 2022 ICube Laboratory, University of Strasbourg
// Copyright 2026 CNR-STIIMA
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <memory>
#include <string>
#include <vector>
#include "controller_interface/controller_interface.hpp"
#include "kdl/chain.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainjnttojacsolver.hpp"

namespace iiwa_joint_admittance_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
class JointAdmittanceController : public controller_interface::ControllerInterface
{
public:
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;
private:
  bool configure_payload();
  void compute_payload_torque();

  std::vector<std::string> joints_, torque_names_;
  std::vector<double> mass_, damping_, stiffness_, deadband_, lower_, upper_;
  std::vector<double> max_velocity_, max_acceleration_;
  std::vector<double> position_, velocity_, reference_, filtered_torque_;
  std::vector<size_t> command_index_, position_index_, torque_index_;
  double filter_time_constant_{0.05}, max_period_{0.02}, max_tracking_error_{0.1};
  // Gravity of a wrist payload unknown to the KUKA controller, removed from tau_ext.
  double payload_mass_{0.0};
  KDL::Vector payload_com_, payload_weight_;
  KDL::Chain chain_;
  std::vector<size_t> chain_to_joint_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  KDL::JntArray chain_q_;
  KDL::Jacobian jacobian_;
  std::vector<double> payload_torque_;
};
}
