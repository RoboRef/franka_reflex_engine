#pragma once

#include <Eigen/Dense>
#include <reflex_controllers/base/panda_controller_base.h>
#include <reflex_controllers/utils/joint_limit_walls.h>

namespace panda_controllers {

struct FrankaAdmittanceControllerPose {
  Eigen::Matrix<double, 7, 1> q;
  Eigen::Matrix<double, 7, 1> qD;
};

struct FrankaAdmittanceControllerParams {
  double alpha{0.05};  // rate scaling: qd_d = alpha * tau_ext
  double deadband{1.0}; // Nm deadband on tau_ext to ignore sensor noise
};

class FrankaAdmittanceController :
    public virtual PandaControllerBase<FrankaAdmittanceControllerParams,
                                       FrankaAdmittanceControllerPose> {
 public:
  virtual ~FrankaAdmittanceController() = default;

 private:
  void computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
                      const FrankaAdmittanceControllerPose& desired,
                      const FrankaAdmittanceControllerParams& p) override;
  FrankaAdmittanceControllerParams defaultParameters() override;
  FrankaAdmittanceControllerPose getCurrentPoseImpl() override;
  bool hasOffsetImpl() override;
  void resetOffset() override;
  void startImpl() override;

  Eigen::Matrix<double, 7, 1> q_d_;
  JointLimitWalls walls_;
};

}  // namespace panda_controllers
