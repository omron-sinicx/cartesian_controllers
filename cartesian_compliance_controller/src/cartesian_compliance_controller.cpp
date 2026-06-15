////////////////////////////////////////////////////////////////////////////////
// Copyright 2019 FZI Research Center for Information Technology
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

//-----------------------------------------------------------------------------
/*!\file    cartesian_compliance_controller.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#include <cartesian_compliance_controller/cartesian_compliance_controller.h>

#include "cartesian_controller_base/Utility.h"
#include "controller_interface/controller_interface.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace cartesian_compliance_controller
{
CartesianComplianceController::CartesianComplianceController()
// Base constructor won't be called in diamond inheritance, so call that
// explicitly
: Base::CartesianControllerBase(),
  MotionBase::CartesianMotionController(),
  ForceBase::CartesianForceController(),
  m_use_parallel_force_position_control(false),
  m_use_selection_matrix_in_gripper_frame(false)
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianComplianceController::on_init()
{
  using TYPE = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  if (MotionBase::on_init() != TYPE::SUCCESS || ForceBase::on_init() != TYPE::SUCCESS)
  {
    return TYPE::ERROR;
  }

  auto_declare<std::string>("compliance_ref_link", "");

  constexpr double default_lin_stiff = 500.0;
  constexpr double default_rot_stiff = 50.0;
  auto_declare<double>("stiffness.trans_x", default_lin_stiff);
  auto_declare<double>("stiffness.trans_y", default_lin_stiff);
  auto_declare<double>("stiffness.trans_z", default_lin_stiff);
  auto_declare<double>("stiffness.rot_x", default_rot_stiff);
  auto_declare<double>("stiffness.rot_y", default_rot_stiff);
  auto_declare<double>("stiffness.rot_z", default_rot_stiff);

  constexpr double default_selection = 0.5;
  auto_declare<double>("stiffness.sel_x", default_selection);
  auto_declare<double>("stiffness.sel_y", default_selection);
  auto_declare<double>("stiffness.sel_z", default_selection);
  auto_declare<double>("stiffness.sel_ax", default_selection);
  auto_declare<double>("stiffness.sel_ay", default_selection);
  auto_declare<double>("stiffness.sel_az", default_selection);

  auto_declare<bool>("stiffness.use_parallel_force_position_control", false);
  auto_declare<bool>("stiffness.use_selection_matrix_in_gripper_frame", false);

  return TYPE::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianComplianceController::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  using TYPE = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  if (MotionBase::on_configure(previous_state) != TYPE::SUCCESS ||
      ForceBase::on_configure(previous_state) != TYPE::SUCCESS)
  {
    return TYPE::ERROR;
  }

  // Make sure compliance link is part of the robot chain
  m_compliance_ref_link = get_node()->get_parameter("compliance_ref_link").as_string();
  if (!Base::robotChainContains(m_compliance_ref_link))
  {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), m_compliance_ref_link
                                                    << " is not part of the kinematic chain from "
                                                    << Base::m_robot_base_link << " to "
                                                    << Base::m_end_effector_link);
    return TYPE::ERROR;
  }

  // Make sure sensor wrenches are interpreted correctly
  ForceBase::setFtSensorReferenceFrame(m_compliance_ref_link);

  m_use_parallel_force_position_control =
    get_node()->get_parameter("stiffness.use_parallel_force_position_control").as_bool();
  m_use_selection_matrix_in_gripper_frame =
    get_node()->get_parameter("stiffness.use_selection_matrix_in_gripper_frame").as_bool();

  updateStiffnessFromParameters();
  updateSelectionMatrixFromParameters();

  m_set_stiffness_server = get_node()->create_service<srv::SetStiffness>(
    std::string(get_node()->get_name()) + "/set_stiffness",
    std::bind(&CartesianComplianceController::setStiffnessCallback, this, std::placeholders::_1,
              std::placeholders::_2));

  return TYPE::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianComplianceController::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  // Base::on_activation(..) will get called twice,
  // but that's fine.
  using TYPE = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  if (MotionBase::on_activate(previous_state) != TYPE::SUCCESS ||
      ForceBase::on_activate(previous_state) != TYPE::SUCCESS)
  {
    return TYPE::ERROR;
  }
  return TYPE::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianComplianceController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  using TYPE = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  if (MotionBase::on_deactivate(previous_state) != TYPE::SUCCESS ||
      ForceBase::on_deactivate(previous_state) != TYPE::SUCCESS)
  {
    return TYPE::ERROR;
  }
  return TYPE::SUCCESS;
}

controller_interface::return_type CartesianComplianceController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Synchronize the internal model and the real robot
  Base::m_ik_solver->synchronizeJointPositions(Base::m_joint_state_pos_handles);

  // Control the robot motion in such a way that the resulting net force
  // vanishes. This internal control needs some simulation time steps.
  for (int i = 0; i < Base::m_iterations; ++i)
  {
    // The internal 'simulation time' is deliberately independent of the outer
    // control cycle.
    auto internal_period = rclcpp::Duration::from_seconds(0.02);

    // Compute the net force
    ctrl::Vector6D error = computeComplianceError();

    // Turn Cartesian error into joint motion
    Base::computeJointControlCmds(error, internal_period);
  }

  // Write final commands to the hardware interface
  Base::writeJointControlCmds();

  return controller_interface::return_type::OK;
}

void CartesianComplianceController::onKinematicChainUpdated()
{
  if (!Base::robotChainContains(m_compliance_ref_link))
  {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), m_compliance_ref_link
                                                    << " is not part of the kinematic chain from "
                                                    << Base::m_robot_base_link << " to "
                                                    << Base::m_end_effector_link);
    return;
  }

  ForceBase::setFtSensorReferenceFrame(m_compliance_ref_link);
}

rcl_interfaces::msg::SetParametersResult CartesianComplianceController::onParametersSet(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result = Base::onParametersSet(parameters);
  if (!result.successful)
  {
    return result;
  }

  bool stiffness_updated = false;
  bool selection_updated = false;

  for (const auto & param : parameters)
  {
    const auto & name = param.get_name();
    if (name == "stiffness.use_parallel_force_position_control")
    {
      m_use_parallel_force_position_control = param.as_bool();
    }
    else if (name == "stiffness.use_selection_matrix_in_gripper_frame")
    {
      m_use_selection_matrix_in_gripper_frame = param.as_bool();
    }
    else if (name.rfind("stiffness.trans_", 0) == 0 || name.rfind("stiffness.rot_", 0) == 0)
    {
      stiffness_updated = true;
    }
    else if (name.rfind("stiffness.sel_", 0) == 0)
    {
      selection_updated = true;
    }
  }

  if (stiffness_updated)
  {
    updateStiffnessFromParameters();
  }
  if (selection_updated)
  {
    updateSelectionMatrixFromParameters();
  }

  return result;
}

void CartesianComplianceController::updateStiffnessFromParameters()
{
  ctrl::Vector6D stiffness_diag;
  stiffness_diag[0] = get_node()->get_parameter("stiffness.trans_x").as_double();
  stiffness_diag[1] = get_node()->get_parameter("stiffness.trans_y").as_double();
  stiffness_diag[2] = get_node()->get_parameter("stiffness.trans_z").as_double();
  stiffness_diag[3] = get_node()->get_parameter("stiffness.rot_x").as_double();
  stiffness_diag[4] = get_node()->get_parameter("stiffness.rot_y").as_double();
  stiffness_diag[5] = get_node()->get_parameter("stiffness.rot_z").as_double();

  std::lock_guard<std::mutex> lock(m_stiffness_mutex);
  m_stiffness = stiffness_diag.asDiagonal();
}

void CartesianComplianceController::updateSelectionMatrixFromParameters()
{
  ctrl::Vector6D selection_diag;
  selection_diag[0] = get_node()->get_parameter("stiffness.sel_x").as_double();
  selection_diag[1] = get_node()->get_parameter("stiffness.sel_y").as_double();
  selection_diag[2] = get_node()->get_parameter("stiffness.sel_z").as_double();
  selection_diag[3] = get_node()->get_parameter("stiffness.sel_ax").as_double();
  selection_diag[4] = get_node()->get_parameter("stiffness.sel_ay").as_double();
  selection_diag[5] = get_node()->get_parameter("stiffness.sel_az").as_double();

  std::lock_guard<std::mutex> lock(m_stiffness_mutex);
  m_selection_matrix = selection_diag.asDiagonal();
}

ctrl::Vector6D CartesianComplianceController::computeComplianceError()
{
  ctrl::Matrix6D stiffness;
  ctrl::Matrix6D selection_matrix;
  bool use_parallel = m_use_parallel_force_position_control;
  bool use_gripper_frame = m_use_selection_matrix_in_gripper_frame;

  {
    std::lock_guard<std::mutex> lock(m_stiffness_mutex);
    stiffness = m_stiffness;
    selection_matrix = m_selection_matrix;
  }

  ctrl::Vector6D net_force;

  if (use_parallel)
  {
    if (use_gripper_frame)
    {
      selection_matrix = Base::displayInBaseLink(selection_matrix, m_compliance_ref_link);
    }

    net_force = selection_matrix * Base::displayInBaseLink(stiffness, m_compliance_ref_link) *
                  MotionBase::computeMotionError() +
                (ctrl::Matrix6D::Identity() - selection_matrix) * ForceBase::computeForceError();
  }
  else
  {
    net_force =
      Base::displayInBaseLink(stiffness, m_compliance_ref_link) * MotionBase::computeMotionError() +
      ForceBase::computeForceError();
  }

  return net_force;
}

void CartesianComplianceController::setStiffnessCallback(
  const std::shared_ptr<srv::SetStiffness::Request> request,
  std::shared_ptr<srv::SetStiffness::Response> response)
{
  if (request->stiffness.size() != 36)
  {
    response->success = false;
    response->message = "Expected 36 stiffness values for a 6x6 matrix.";
    return;
  }

  ctrl::Matrix6D stiffness;
  for (int i = 0; i < 6; ++i)
  {
    for (int j = 0; j < 6; ++j)
    {
      stiffness(i, j) = request->stiffness[static_cast<size_t>(i * 6 + j)];
    }
  }

  {
    std::lock_guard<std::mutex> lock(m_stiffness_mutex);
    m_stiffness = stiffness;
  }

  response->success = true;
  response->message = "";
}

}  // namespace cartesian_compliance_controller

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(cartesian_compliance_controller::CartesianComplianceController,
                       controller_interface::ControllerInterface)
