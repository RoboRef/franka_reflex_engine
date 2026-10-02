#pragma once

#include <Eigen/Dense>
#include <reflex_controllers/base/panda_controller_base.h>
#include <reflex_controllers/controllers/comless_panda_cartesian_impedance_controller.h>

namespace panda_controllers {

class ComlessWipingController :
    public virtual PandaControllerBase<PandaCartesianImpedanceControllerParams,
                                       PandaCartesianImpedanceControllerPose> {
 public:
  virtual ~ComlessWipingController() = default;

 private:
  void computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
      const PandaCartesianImpedanceControllerPose& desired,
      const PandaCartesianImpedanceControllerParams& p) override final;
  PandaCartesianImpedanceControllerParams defaultParameters() override final;
  PandaCartesianImpedanceControllerPose getCurrentPoseImpl() override final;
  bool hasOffsetImpl() override final;
  void resetOffset() override final;
  void startImpl() override;

  double time_traj_{0.0};
  double target_z_force_{-5.0}; // Max 5 N downward force (Z axis)
  double current_z_force_{0.0};
  double f_error_integral_z_{0.0};
  
  Eigen::Vector3d initial_position_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond initial_orientation_{Eigen::Quaterniond::Identity()};
  Eigen::Matrix<double, 6, 1> initial_f_ext_hat_{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 7, 1> initial_tau_ext_{Eigen::Matrix<double, 7, 1>::Zero()};
  bool initialized_start_pose_{false};
};

}
