// Copyright 2022 ICube Laboratory, University of Strasbourg
// Copyright 2026 CNR-STIIMA
// SPDX-License-Identifier: Apache-2.0
#include "admittance_controller/admittance_controller.hpp"
#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace iiwa_joint_admittance_controller
{
CallbackReturn JointAdmittanceController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::vector<std::string>>("torque_interfaces", {});
    for (const auto * name : {"mass", "damping", "stiffness", "torque_deadband",
      "joint_lower_limits", "joint_upper_limits", "max_velocity", "max_acceleration"}) {
      auto_declare<std::vector<double>>(name, {});
    }
    auto_declare<double>("filter_time_constant", 0.05);
    auto_declare<double>("max_period", 0.02);
    auto_declare<double>("max_tracking_error", 0.1);
    auto_declare<double>("payload_mass", 0.0);
    auto_declare<std::vector<double>>("payload_com", {0.0, 0.0, 0.0});
    auto_declare<std::string>("payload_frame", "");
    auto_declare<std::string>("gravity_frame", "world");
    auto_declare<std::vector<double>>("gravity", {0.0, 0.0, -9.81});
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointAdmittanceController::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    joints_ = get_node()->get_parameter("joints").as_string_array();
    torque_names_ = get_node()->get_parameter("torque_interfaces").as_string_array();
    if (joints_.size() != 7 || torque_names_.size() != joints_.size() ||
      std::set<std::string>(joints_.begin(), joints_.end()).size() != joints_.size() ||
      std::set<std::string>(torque_names_.begin(), torque_names_.end()).size() != joints_.size()) {
      throw std::runtime_error("Specify seven unique joints and seven matching torque_interfaces");
    }
    auto read = [this](const char * name, std::vector<double> & values, bool positive) {
      values = get_node()->get_parameter(name).as_double_array();
      if (values.size() != joints_.size()) {throw std::runtime_error(std::string(name) + ": expected seven values");}
      for (double v : values) {
        if (!std::isfinite(v) || (positive ? v <= 0.0 : v < 0.0)) {
          throw std::runtime_error(std::string(name) + ": invalid value");
        }
      }
    };
    read("mass", mass_, true);
    read("damping", damping_, true);
    read("stiffness", stiffness_, false);
    read("torque_deadband", deadband_, false);
    read("max_velocity", max_velocity_, true);
    read("max_acceleration", max_acceleration_, true);
    lower_ = get_node()->get_parameter("joint_lower_limits").as_double_array();
    upper_ = get_node()->get_parameter("joint_upper_limits").as_double_array();
    if (lower_.size() != joints_.size() || upper_.size() != joints_.size()) {
      throw std::runtime_error("Specify seven lower and upper joint limits");
    }
    for (size_t i = 0; i < joints_.size(); ++i) {
      if (!std::isfinite(lower_[i]) || !std::isfinite(upper_[i]) || lower_[i] >= upper_[i]) {
        throw std::runtime_error("Invalid joint limits");
      }
    }
    filter_time_constant_ = get_node()->get_parameter("filter_time_constant").as_double();
    max_period_ = get_node()->get_parameter("max_period").as_double();
    max_tracking_error_ = get_node()->get_parameter("max_tracking_error").as_double();
    if (!std::isfinite(filter_time_constant_) || filter_time_constant_ < 0 ||
      !std::isfinite(max_period_) || max_period_ <= 0 ||
      !std::isfinite(max_tracking_error_) || max_tracking_error_ <= 0) {
      throw std::runtime_error("Invalid filter or watchdog parameters");
    }
    position_.resize(7); velocity_.resize(7); reference_.resize(7); filtered_torque_.resize(7);
    payload_torque_.assign(7, 0.0);
    if (!configure_payload()) {return CallbackReturn::ERROR;}
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

bool JointAdmittanceController::configure_payload()
{
  auto node = get_node();
  payload_mass_ = node->get_parameter("payload_mass").as_double();
  const auto com = node->get_parameter("payload_com").as_double_array();
  const auto gravity = node->get_parameter("gravity").as_double_array();
  if (!std::isfinite(payload_mass_) || payload_mass_ < 0 || com.size() != 3 || gravity.size() != 3 ||
    !std::all_of(com.begin(), com.end(), [](double v) {return std::isfinite(v);}) ||
    !std::all_of(gravity.begin(), gravity.end(), [](double v) {return std::isfinite(v);})) {
    throw std::runtime_error("payload_mass must be >= 0; payload_com and gravity need three values");
  }
  if (payload_mass_ == 0.0) {
    fk_solver_.reset(); jac_solver_.reset();
    return true;
  }
  payload_com_ = KDL::Vector(com[0], com[1], com[2]);
  payload_weight_ = payload_mass_ * KDL::Vector(gravity[0], gravity[1], gravity[2]);
  const auto root = node->get_parameter("gravity_frame").as_string();
  const auto tip = node->get_parameter("payload_frame").as_string();
  if (root.empty() || tip.empty()) {throw std::runtime_error("Specify gravity_frame and payload_frame");}

  // Jazzy supplies the same URDF used by controller_manager directly to controllers.
  const auto & description = get_robot_description();
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(description, tree) || !tree.getChain(root, tip, chain_)) {
    throw std::runtime_error("Cannot build a KDL chain from " + root + " to " + tip);
  }
  chain_to_joint_.clear();
  for (const auto & segment : chain_.segments) {
    if (segment.getJoint().getType() == KDL::Joint::None) {continue;}
    auto it = std::find(joints_.begin(), joints_.end(), segment.getJoint().getName());
    if (it == joints_.end()) {
      throw std::runtime_error("Chain joint " + segment.getJoint().getName() + " is not controlled");
    }
    chain_to_joint_.push_back(static_cast<size_t>(it - joints_.begin()));
  }
  if (chain_to_joint_.size() != joints_.size()) {
    throw std::runtime_error("The chain to payload_frame must contain exactly the seven joints");
  }
  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
  jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
  chain_q_.resize(chain_.getNrOfJoints());
  jacobian_.resize(chain_.getNrOfJoints());
  RCLCPP_INFO(node->get_logger(), "Compensating %.3f kg payload on %s", payload_mass_, tip.c_str());
  return true;
}

// tau = J_com^T * m*g: the joint torque the payload weight produces, which FRI reports as
// external torque when the load is not declared on the KUKA side.
void JointAdmittanceController::compute_payload_torque()
{
  if (!jac_solver_) {return;}
  for (size_t c = 0; c < chain_to_joint_.size(); ++c) {
    chain_q_(c) = state_interfaces_[position_index_[chain_to_joint_[c]]].get_value();
  }
  KDL::Frame tip;
  fk_solver_->JntToCart(chain_q_, tip);
  jac_solver_->JntToJac(chain_q_, jacobian_);
  jacobian_.changeRefPoint(tip.M * payload_com_);
  for (size_t c = 0; c < chain_to_joint_.size(); ++c) {
    payload_torque_[chain_to_joint_[c]] = jacobian_(0, c) * payload_weight_.x() +
      jacobian_(1, c) * payload_weight_.y() + jacobian_(2, c) * payload_weight_.z();
  }
}

controller_interface::InterfaceConfiguration
JointAdmittanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration c;
  c.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {c.names.push_back(joint + "/position");}
  return c;
}
controller_interface::InterfaceConfiguration
JointAdmittanceController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration c;
  c.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {c.names.push_back(joint + "/position");}
  c.names.insert(c.names.end(), torque_names_.begin(), torque_names_.end());
  return c;
}

CallbackReturn JointAdmittanceController::on_activate(const rclcpp_lifecycle::State &)
{
  command_index_.clear(); position_index_.clear(); torque_index_.clear();
  auto find = [](const auto & interfaces, const std::string & name) {
    for (size_t i = 0; i < interfaces.size(); ++i) {
      if (interfaces[i].get_name() == name) {return i;}
    }
    throw std::runtime_error("Missing interface: " + name);
  };
  try {
    for (size_t i = 0; i < joints_.size(); ++i) {
      command_index_.push_back(find(command_interfaces_, joints_[i] + "/position"));
      position_index_.push_back(find(state_interfaces_, joints_[i] + "/position"));
      torque_index_.push_back(find(state_interfaces_, torque_names_[i]));
      double q = state_interfaces_[position_index_[i]].get_value();
      double tau = state_interfaces_[torque_index_[i]].get_value();
      if (!std::isfinite(q) || !std::isfinite(tau) || q < lower_[i] || q > upper_[i]) {
        throw std::runtime_error("Invalid initial position or torque");
      }
      reference_[i] = position_[i] = q;
      velocity_[i] = filtered_torque_[i] = 0.0;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", e.what());
    return CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < joints_.size(); ++i) {
    command_interfaces_[command_index_[i]].set_value(position_[i]);
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointAdmittanceController::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Keep the last bounded position command; never send zero on a position interface.
  std::fill(velocity_.begin(), velocity_.end(), 0.0);
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type JointAdmittanceController::update(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  const double dt = period.seconds();
  bool valid = std::isfinite(dt) && dt > 0 && dt <= max_period_;
  // Validate all samples before writing any command. On fault retain the last command.
  for (size_t i = 0; i < joints_.size(); ++i) {
    const double q = state_interfaces_[position_index_[i]].get_value();
    const double tau = state_interfaces_[torque_index_[i]].get_value();
    valid = valid && std::isfinite(q) && std::isfinite(tau) &&
      q >= lower_[i] && q <= upper_[i] && std::abs(q - position_[i]) <= max_tracking_error_;
  }
  if (!valid) {
    std::fill(velocity_.begin(), velocity_.end(), 0.0);
    return controller_interface::return_type::ERROR;
  }
  const double alpha = filter_time_constant_ == 0 ? 1.0 : -std::expm1(-dt / filter_time_constant_);
  compute_payload_torque();
  for (size_t i = 0; i < joints_.size(); ++i) {
    const double tau = state_interfaces_[torque_index_[i]].get_value() - payload_torque_[i];
    filtered_torque_[i] += alpha * (tau - filtered_torque_[i]);
    const double force = std::copysign(std::max(0.0, std::abs(filtered_torque_[i]) - deadband_[i]), filtered_torque_[i]);
    const double drive = force + stiffness_[i] * (reference_[i] - position_[i]);
    // Exact velocity solution for constant drive over one cycle; stable for positive damping.
    const double target_velocity = drive / damping_[i] +
      (velocity_[i] - drive / damping_[i]) * std::exp(-damping_[i] * dt / mass_[i]);
    const double dv = std::clamp(target_velocity - velocity_[i],
      -max_acceleration_[i] * dt, max_acceleration_[i] * dt);
    velocity_[i] = std::clamp(velocity_[i] + dv, -max_velocity_[i], max_velocity_[i]);
    const double next = position_[i] + dt * velocity_[i];
    position_[i] = std::clamp(next, lower_[i], upper_[i]);
    if (position_[i] != next) {velocity_[i] = 0;}
    command_interfaces_[command_index_[i]].set_value(position_[i]);
  }
  return controller_interface::return_type::OK;
}
}
PLUGINLIB_EXPORT_CLASS(iiwa_joint_admittance_controller::JointAdmittanceController,
  controller_interface::ControllerInterface)
