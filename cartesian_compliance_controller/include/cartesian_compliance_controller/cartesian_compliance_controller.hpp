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
    : Base::CartesianControllerBase(),
      MotionBase::CartesianMotionController(),
      ForceBase::CartesianForceController() {}

template <class HardwareInterface>
bool CartesianComplianceController<HardwareInterface>::init(
    HardwareInterface* hw, ros::NodeHandle& nh) {
  // Only one of them will call Base::init(hw,nh);
  MotionBase::init(hw, nh);
  ForceBase::init(hw, nh);
  m_use_parallel_force_position_control = false;

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
    const ros::Time& time) {
  // Base::starting(time) will get called twice,
  // but that's fine.
  MotionBase::starting(time);
  ForceBase::starting(time);
}

template <class HardwareInterface>
void CartesianComplianceController<HardwareInterface>::stopping(
    const ros::Time& time) {
  MotionBase::stopping(time);
  ForceBase::stopping(time);
}

template <class HardwareInterface>
void CartesianComplianceController<HardwareInterface>::update(
    const ros::Time& time, const ros::Duration& period) {
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
  std::cout << "m_selection_matrix: " << std::endl
            << m_selection_matrix << std::endl;
  bool powder_grounding_flag = true;
  // bool powder_grounding_flag = false;

  ctrl::Vector6D net_force;
  if (!m_use_parallel_force_position_control)
    net_force =

        // Spring force in base orientation
        Base::displayInBaseLink(m_stiffness, m_compliance_ref_link) *
            MotionBase::computeMotionError()

        // Sensor and target force in base orientation
        + ForceBase::computeForceError();
  else if (powder_grounding_flag) {
    auto pose = Base::m_ik_solver->getEndEffectorPose();
    std::cout << "xyz: " << pose.p.x() << ", " << pose.p.y() << ", "
              << pose.p.z() << std::endl;
    ctrl::Vector3D ee_pos;
    ee_pos << pose.p.x(), pose.p.y(), pose.p.z();
    std::cout << "ee_pos:" << std::endl << ee_pos << std::endl;
    ctrl::Vector3D center;
    // center << 0.0, 0.5, 0.04;
    // center << 0.0, 0.5, 0.081;
    // center << 0.0, 0.5, 0.05;
    center << -0.17, 0.51, 0.0755;
    std::cout << "center:" << std::endl << center << std::endl;
    ctrl::Vector3D normal_vec;
    // normal_vec = (ee_pos - center).normalized();
    normal_vec = (center - ee_pos).normalized();
    // normal_vec = (center - ee_pos).normalized();
    std::cout << "normal_vec:" << std::endl << normal_vec << std::endl;
    ctrl::Vector3D x_basis;
    x_basis << 1.0, 0.0, 0.0;
    ctrl::Vector3D u_x = x_basis - x_basis.dot(normal_vec) * normal_vec;
    std::cout << "u_x:" << std::endl << u_x << std::endl;
    u_x.normalize();
    ctrl::Vector3D y_basis;
    y_basis << 0.0, 1.0, 0.0;
    ctrl::Vector3D u_y =
        y_basis - y_basis.dot(normal_vec) * normal_vec - y_basis.dot(u_x) * u_x;
    u_y.normalize();
    std::cout << "u_y:" << std::endl << u_y << std::endl;
    ctrl::Matrix3D R_base2surface;
    R_base2surface << u_x(0), u_y(0), normal_vec(0), u_x(1), u_y(1),
        normal_vec(1), u_x(2), u_y(2), normal_vec(2);
    std::cout << "R_base2surface:" << std::endl << R_base2surface << std::endl;
    // ctrl::Matrix3D R_check = R_base2surface * R_base2surface.transpose();
    // std::cout << "R_check:" << std::endl << R_check << std::endl;
    double R_det = R_base2surface.determinant();
    std::cout << "R_det:" << std::endl << R_det << std::endl;
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
    std::cout << "Adjoint_T:" << std::endl << Adjoint_T << std::endl;
    ctrl::Matrix6D Adjoint_T_inv;
    ctrl::Matrix3D R_base2surface_T = R_base2surface.transpose();
    // Adjoint_T_inv << R_base2surface_T, ctrl::Matrix3D::Zero(3, 3),
    //                   pxR.transpose(),             R_base2surface_T;
    Adjoint_T_inv << R_base2surface_T, ctrl::Matrix3D::Zero(3, 3),
        ctrl::Matrix3D::Zero(3, 3), R_base2surface_T;
    std::cout << "Adjoint_T_inv:" << std::endl << Adjoint_T_inv << std::endl;
    ctrl::Matrix6D Adjoint_T_inv_check;
    Adjoint_T_inv_check = Adjoint_T * Adjoint_T_inv;
    std::cout << "Adjoint_T_inv_check:" << std::endl
              << Adjoint_T_inv_check << std::endl;

    ctrl::Matrix6D m_selection_matrix_pd =
        Adjoint_T_inv * m_selection_matrix * Adjoint_T;
    std::cout << "m_selection_matrix_pd: " << std::endl
              << m_selection_matrix_pd << std::endl;

    // ctrl::Matrix6D m_selection_matrix_pd_check;
    // m_selection_matrix_pd_check =
    //     m_selection_matrix_pd +
    //     (ctrl::Matrix6D::Identity() - m_selection_matrix_pd);
    // std::cout << "m_selection_matrix_pd_check: " << std::endl <<
    // m_selection_matrix_pd_check << std::endl;

    auto current_positions = Base::m_ik_solver->getPositions();
    auto current_velocity = Base::m_ik_solver->getVelocity();
    KDL::JntSpaceInertiaMatrix jnt_space_inertia;
    m_jnt_space_inertia_solver->JntToMass(current_positions, jnt_space_inertia);
    std::cout << "jnt_space_inertia: " << std::endl;
    std::cout << jnt_space_inertia.data << std::endl;
    KDL::Jacobian jnt_jacobian;
    m_jnt_jacobian_solver->JntToJac(current_positions, jnt_jacobian);
    std::cout << "jnt_jacobian: " << std::endl;
    std::cout << jnt_jacobian.data << std::endl;

    // ctrl::Vector6D sensor_wrench;
    // sensor_wrench = ForceBase::getFTSensorWrench();
    // std::cout << "sensor_wrench: " << std::endl;
    // std::cout << sensor_wrench << std::endl;

    net_force =

        // Position controller: PID gains scale by m_stiffness
        m_selection_matrix_pd *
            Base::displayInBaseLink(m_stiffness, m_compliance_ref_link) *
            MotionBase::computeMotionError()

        // Sensor and target force in base orientation
        + ((ctrl::Matrix6D::Identity() - m_selection_matrix_pd) *
           ForceBase::computeForceError());
  } else  // Add a selection matrix to allow finer control of which control to
          // use for each
  {
    net_force =

        // Position controller: PID gains scale by m_stiffness
        m_selection_matrix *
            Base::displayInBaseLink(m_stiffness, m_compliance_ref_link) *
            MotionBase::computeMotionError()

        // Sensor and target force in base orientation
        + ((ctrl::Matrix6D::Identity() - m_selection_matrix) *
           ForceBase::computeForceError());
  }

  return net_force;
}

template <class HardwareInterface>
void CartesianComplianceController<
    HardwareInterface>::dynamicReconfigureCallback(ComplianceConfig& config,
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

  tmp[0] = config.sel_x;
  tmp[1] = config.sel_y;
  tmp[2] = config.sel_z;
  tmp[3] = config.sel_ax;
  tmp[4] = config.sel_ay;
  tmp[5] = config.sel_az;
  m_selection_matrix = tmp.asDiagonal();
}

}  // namespace cartesian_compliance_controller

#endif
