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
/*!\file    cartesian_compliance_controller.hpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#ifndef CARTESIAN_COMPLIANCE_CONTROLLER_HPP_INCLUDED
#define CARTESIAN_COMPLIANCE_CONTROLLER_HPP_INCLUDED

// Project
#include <cartesian_compliance_controller/cartesian_compliance_controller.h>

// Other
#include <algorithm>
#include <map>

namespace cartesian_compliance_controller {

template <class HardwareInterface>
CartesianComplianceController<
    HardwareInterface>::CartesianComplianceController()
    // Base constructor won't be called in diamond inheritance, so call that
    // explicitly
    : Base::CartesianControllerBase(), MotionBase::CartesianMotionController(),
      ForceBase::CartesianForceController() {}

template <class HardwareInterface>
bool CartesianComplianceController<HardwareInterface>::init(
    HardwareInterface *hw, ros::NodeHandle &nh) {
  // Only one of them will call Base::init(hw,nh);
  MotionBase::init(hw, nh);
  ForceBase::init(hw, nh);
  m_use_parallel_force_position_control = false;
  m_use_selection_matrix_in_gripper_frame = false;

  if (!nh.getParam("compliance_ref_link", m_compliance_ref_link)) {
    ROS_ERROR_STREAM("Failed to load "
                     << nh.getNamespace() + "/compliance_ref_link"
                     << " from parameter server");
    return false;
  }

  // Make sure compliance link is part of the robot chain
  if (!Base::robotChainContains(m_compliance_ref_link)) {
    ROS_ERROR_STREAM(m_compliance_ref_link
                     << " is not part of the kinematic chain from "
                     << Base::m_robot_base_link << " to "
                     << Base::m_end_effector_link);
    return false;
  }

  // Make sure sensor wrenches are interpreted correctly
  ForceBase::setFtSensorReferenceFrame(m_compliance_ref_link);

  // Connect dynamic reconfigure and overwrite the default values with values
  // on the parameter server. This is done automatically if parameters with
  // the according names exist.
  m_callback_type =
      std::bind(&CartesianComplianceController<
                    HardwareInterface>::dynamicReconfigureCallback,
                this, std::placeholders::_1, std::placeholders::_2);

  m_dyn_conf_server.reset(new dynamic_reconfigure::Server<ComplianceConfig>(
      ros::NodeHandle(nh.getNamespace() + "/stiffness")));
  m_dyn_conf_server->setCallback(m_callback_type);

  // KDL::Chain chain = Base::m_ik_solver->getChain();
  m_jnt_jacobian_solver.reset(
      new KDL::ChainJntToJacSolver(Base::m_ik_solver->getChain()));
  m_jnt_space_inertia_solver.reset(new KDL::ChainDynParam(
      Base::m_ik_solver->getChain(), KDL::Vector::Zero()));

  return true;
}

template <class HardwareInterface>
void CartesianComplianceController<HardwareInterface>::starting(
    const ros::Time &time) {
  // Base::starting(time) will get called twice,
  // but that's fine.
  MotionBase::starting(time);
  ForceBase::starting(time);
}

template <class HardwareInterface>
void CartesianComplianceController<HardwareInterface>::stopping(
    const ros::Time &time) {
  MotionBase::stopping(time);
  ForceBase::stopping(time);
}

template <class HardwareInterface>
void CartesianComplianceController<HardwareInterface>::update(
    const ros::Time &time, const ros::Duration &period) {
  // Synchronize the internal model and the real robot
  Base::m_ik_solver->synchronizeJointPositions(Base::m_joint_handles);

  // Control the robot motion in such a way that the resulting net force
  // vanishes. This internal control needs some simulation time steps.
  for (int i = 0; i < Base::m_iterations; ++i) {
    // The internal 'simulation time' is deliberately independent of the outer
    // control cycle.
    ros::Duration internal_period(0.02);

    // Compute the net force
    ctrl::Vector6D error = computeComplianceError();

    // Turn Cartesian error into joint motion
    Base::computeJointControlCmds(error, internal_period);
  }

  // Write final commands to the hardware interface
  Base::writeJointControlCmds();
}

template <class HardwareInterface>
ctrl::Vector6D
CartesianComplianceController<HardwareInterface>::computeComplianceError() {
  ctrl::Matrix6D selection_matrix;
  ctrl::Vector6D net_force;

  // First check whether we use parallel control (requires selection matrix)
  if (m_use_parallel_force_position_control) {
      if (m_use_selection_matrix_in_gripper_frame) {
        selection_matrix = Base::displayInBaseLink(m_selection_matrix, m_compliance_ref_link);
      } else {
        selection_matrix = m_selection_matrix;
      }
      // Position controller: PID gains scale by m_stiffness
      net_force = selection_matrix *
              Base::displayInBaseLink(m_stiffness, m_compliance_ref_link) *
              MotionBase::computeMotionError()
          // Sensor and target force in base orientation
          + (ctrl::Matrix6D::Identity() - selection_matrix) *
             ForceBase::computeForceError();
  } else {
      // use original spring-mass-damper control
      net_force =

        // Spring force in base orientation
        Base::displayInBaseLink(m_stiffness, m_compliance_ref_link) *
            MotionBase::computeMotionError()

        // Sensor and target force in base orientation
        + ForceBase::computeForceError();
  }

  return net_force;
}

template <class HardwareInterface>
void CartesianComplianceController<HardwareInterface>::apply_selection_matrix_gripper_frame(
    ctrl::Vector6D pos_error, ctrl::Vector6D force_error,
    ctrl::Vector6D *pos_error_sel, ctrl::Vector6D *force_error_sel,
    ctrl::Matrix6D selection_matrix_gripper, ctrl::Matrix3D R_eef_to_base) 
{
    // Convert errors from reference frame to gripper frame
    ctrl::Matrix6D bigR = ctrl::Matrix6D::Zero(6, 6);
    bigR.topLeftCorner(3, 3) = R_eef_to_base;
    bigR.bottomRightCorner(3, 3) = R_eef_to_base;

    ctrl::Matrix6D invBigR = ctrl::Matrix6D::Zero(6, 6);
    invBigR.topLeftCorner(3, 3) = R_eef_to_base.transpose();
    invBigR.bottomRightCorner(3, 3) = R_eef_to_base.transpose();

    // Transform errors to gripper frame (rotation only, no translation)
    ctrl::Vector6D pose_error_gripper = invBigR * pos_error;
    ctrl::Vector6D ft_error_gripper = invBigR * force_error;

    // Apply selection matrix in gripper frame
    ctrl::Vector6D selected_pose_error_gripper = selection_matrix_gripper * pose_error_gripper;
    ctrl::Vector6D selected_ft_error_gripper = (ctrl::Matrix6D::Identity() - selection_matrix_gripper) * ft_error_gripper;

    // Convert selected errors back to reference frame
    *pos_error_sel = bigR * selected_pose_error_gripper;
    *force_error_sel = bigR * selected_ft_error_gripper;
}

template <class HardwareInterface>
void CartesianComplianceController<
    HardwareInterface>::dynamicReconfigureCallback(ComplianceConfig &config,
                                                   uint32_t level) {
  ctrl::Vector6D tmp;
  tmp[0] = config.trans_x;
  tmp[1] = config.trans_y;
  tmp[2] = config.trans_z;
  tmp[3] = config.rot_x;
  tmp[4] = config.rot_y;
  tmp[5] = config.rot_z;
  m_stiffness = tmp.asDiagonal();

  m_use_parallel_force_position_control =
      config.use_parallel_force_position_control;

  m_use_selection_matrix_in_gripper_frame =
      config.use_selection_matrix_in_gripper_frame;

  tmp[0] = config.sel_x;
  tmp[1] = config.sel_y;
  tmp[2] = config.sel_z;
  tmp[3] = config.sel_ax;
  tmp[4] = config.sel_ay;
  tmp[5] = config.sel_az;
  m_selection_matrix = tmp.asDiagonal();
}

} // namespace cartesian_compliance_controller

#endif
