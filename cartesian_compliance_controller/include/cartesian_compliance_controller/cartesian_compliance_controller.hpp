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
  m_use_parallel_force_position_control_in_gripper_frame = false;
  m_use_parallel_force_position_control_powder_grinding = false;

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
  ctrl::Vector6D pose_error, pose_error_sel;
  ctrl::Vector6D ft_error, ft_error_sel;
  ctrl::Matrix3D eef_to_base;
  ctrl::Vector6D net_force;
  if (!m_use_parallel_force_position_control)
    net_force =

        // Spring force in base orientation
        Base::displayInBaseLink(m_stiffness, m_compliance_ref_link) *
            MotionBase::computeMotionError()

        // Sensor and target force in base orientation
        + ForceBase::computeForceError();
  else // Add a selection matrix to allow finer control of which control to
       // use for each
  {
    if (m_use_parallel_force_position_control_in_gripper_frame) {
      // Selection matrix assumed to be in robot's end-effector frame

      // get error in base frame
      pose_error = MotionBase::computeMotionError();
      ft_error = ForceBase::computeForceError();

      // transformation of gripper in base frame
      KDL::Frame transform_kdl;
      Base::m_forward_kinematics_solver->JntToCart(
          Base::m_ik_solver->getPositions(), transform_kdl,
          m_compliance_ref_link);
      // Adjust format from kdl to matrix3d
      eef_to_base << transform_kdl.M.data[0], transform_kdl.M.data[1],
          transform_kdl.M.data[2], transform_kdl.M.data[3],
          transform_kdl.M.data[4], transform_kdl.M.data[5],
          transform_kdl.M.data[6], transform_kdl.M.data[7],
          transform_kdl.M.data[8];

      // changing the error such that the selection matrix from the gripper
      // frame is applied in gripper frame
      apply_selection_matrix_gripper_frame(pose_error, ft_error,
                                           &pose_error_sel, &ft_error_sel,
                                           m_selection_matrix, eef_to_base);

      // stiffness scaling
      net_force = m_stiffness * pose_error_sel + ft_error_sel;

    } else if (m_use_parallel_force_position_control_powder_grinding) {
      // selection matrix computed from estimated normal of bowl's surface
      ctrl::Matrix6D m_selection_matrix_pd;
      compute_selection_matrix_from_normal_surface(&m_selection_matrix_pd);

      net_force =
          // Position controller:   PID gains scale by m_stiffness
          m_selection_matrix_pd *
              Base::displayInBaseLink(m_stiffness, m_compliance_ref_link) *
              MotionBase::computeMotionError()

          // Sensor and target force in base orientation
          + ((ctrl::Matrix6D::Identity() - m_selection_matrix_pd) *
             ForceBase::computeForceError());

    } else {
      // Selection matrix assumed to be in robot's base frame
      net_force =

          // Position controller: PID gains scale by m_stiffness
          m_selection_matrix *
              Base::displayInBaseLink(m_stiffness, m_compliance_ref_link) *
              MotionBase::computeMotionError()

          // Sensor and target force in base orientation
          + ((ctrl::Matrix6D::Identity() - m_selection_matrix) *
             ForceBase::computeForceError());
    }
  }

  return net_force;
}

// helper function for transforming

template <class HardwareInterface>
void CartesianComplianceController<HardwareInterface>::
    apply_selection_matrix_gripper_frame(
        ctrl::Vector6D pos_error_ref, ctrl::Vector6D force_error_ref,
        ctrl::Vector6D *pos_error_sel, ctrl::Vector6D *force_error_sel,
        ctrl::Matrix6D selection_matrix_gripper,
        ctrl::Matrix3D R_gripper_to_ref) {
  // Convert errors from reference frame to gripper frame
  ctrl::Matrix6D bigR = ctrl::Matrix6D::Zero(6, 6);
  bigR.topLeftCorner(3, 3) = R_gripper_to_ref;
  bigR.bottomRightCorner(3, 3) = R_gripper_to_ref;

  ctrl::Matrix6D invBigR = ctrl::Matrix6D::Zero(6, 6);
  invBigR.topLeftCorner(3, 3) = R_gripper_to_ref.transpose();
  invBigR.bottomRightCorner(3, 3) = R_gripper_to_ref.transpose();

  ctrl::Vector6D pose_error_gripper =
      invBigR * pos_error_ref; // transpose,not inv since its just rotation,
                               // without translation
  ctrl::Vector6D ft_error_gripper = invBigR * force_error_ref;

  // Apply selection matrix in gripper frame
  ctrl::Vector6D selected_pose_error_gripper =
      selection_matrix_gripper * pose_error_gripper;
  ctrl::Vector6D selected_ft_error_gripper =
      (ctrl::Matrix6D::Identity() - selection_matrix_gripper) *
      ft_error_gripper;

  // Convert selected errors back to reference frame
  *pos_error_sel = bigR * selected_pose_error_gripper;
  *force_error_sel = bigR * selected_ft_error_gripper;
}

template <class HardwareInterface>
void CartesianComplianceController<HardwareInterface>::
    compute_selection_matrix_from_normal_surface(
        ctrl::Matrix6D *m_selection_matrix_pd) {
  auto pose = Base::m_ik_solver->getEndEffectorPose();
  // std::cout << "xyz: " << pose.p.x() << ", " << pose.p.y() << ", "
  //           << pose.p.z() << std::endl;
  ctrl::Vector3D ee_pos;
  ee_pos << pose.p.x(), pose.p.y(), pose.p.z();
  // std::cout << "ee_pos:" << std::endl << ee_pos << std::endl;
  ctrl::Vector3D center;
  // center << 0.0, 0.5, 0.04;
  // center << 0.0, 0.5, 0.081;
  // center << 0.0, 0.5, 0.05;
  center << -0.17, 0.51, 0.0755;
  // std::cout << "center:" << std::endl << center << std::endl;
  ctrl::Vector3D normal_vec;
  // normal_vec = (ee_pos - center).normalized();
  normal_vec = (center - ee_pos).normalized();
  // normal_vec = (center - ee_pos).normalized();
  // std::cout << "normal_vec:" << std::endl << normal_vec << std::endl;
  ctrl::Vector3D x_basis;
  x_basis << 1.0, 0.0, 0.0;
  ctrl::Vector3D u_x = x_basis - x_basis.dot(normal_vec) * normal_vec;
  // std::cout << "u_x:" << std::endl << u_x << std::endl;
  u_x.normalize();
  ctrl::Vector3D y_basis;
  y_basis << 0.0, 1.0, 0.0;
  ctrl::Vector3D u_y =
      y_basis - y_basis.dot(normal_vec) * normal_vec - y_basis.dot(u_x) * u_x;
  u_y.normalize();
  // std::cout << "u_y:" << std::endl << u_y << std::endl;
  ctrl::Matrix3D R_base2surface;
  R_base2surface << u_x(0), u_y(0), normal_vec(0), u_x(1), u_y(1),
      normal_vec(1), u_x(2), u_y(2), normal_vec(2);
  // std::cout << "R_base2surface:" << std::endl << R_base2surface <<
  // std::endl; ctrl::Matrix3D R_check = R_base2surface *
  // R_base2surface.transpose(); std::cout << "R_check:" << std::endl <<
  // R_check << std::endl;
  double R_det = R_base2surface.determinant();
  // std::cout << "R_det:" << std::endl << R_det << std::endl;
  // ctrl::Matrix3D px;
  // px << 0, -pose.p.z(), pose.p.y(), pose.p.z(), 0, -pose.p.x(),
  // -pose.p.y(),
  //     pose.p.x(), 0;
  // std::cout << "px:" << std::endl << px << std::endl;
  // ctrl::Matrix3D pxR;
  // pxR = px * R_base2surface;
  // std::cout << "pxR:" << std::endl << pxR << std::endl;
  ctrl::Matrix6D Adjoint_T;
  // Adjoint_T << R_base2surface, ctrl::Matrix3D::Zero(3, 3),
  //                         pxR,             R_base2surface;
  Adjoint_T << R_base2surface, ctrl::Matrix3D::Zero(3, 3),
      ctrl::Matrix3D::Zero(3, 3), R_base2surface;
  // std::cout << "Adjoint_T:" << std::endl << Adjoint_T << std::endl;
  ctrl::Matrix6D Adjoint_T_inv;
  ctrl::Matrix3D R_base2surface_T = R_base2surface.transpose();
  // Adjoint_T_inv << R_base2surface_T, ctrl::Matrix3D::Zero(3, 3),
  //                   pxR.transpose(),             R_base2surface_T;
  Adjoint_T_inv << R_base2surface_T, ctrl::Matrix3D::Zero(3, 3),
      ctrl::Matrix3D::Zero(3, 3), R_base2surface_T;
  // std::cout << "Adjoint_T_inv:" << std::endl << Adjoint_T_inv <<
  // std::endl;
  ctrl::Matrix6D Adjoint_T_inv_check;
  Adjoint_T_inv_check = Adjoint_T * Adjoint_T_inv;
  // std::cout << "Adjoint_T_inv_check:" << std::endl
  //           << Adjoint_T_inv_check << std::endl;

  *m_selection_matrix_pd = Adjoint_T_inv * m_selection_matrix * Adjoint_T;
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

  m_use_parallel_force_position_control_in_gripper_frame =
      config.use_parallel_force_position_control_in_gripper_frame;

  m_use_parallel_force_position_control_powder_grinding =
      config.use_parallel_force_position_control_powder_grinding;

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
