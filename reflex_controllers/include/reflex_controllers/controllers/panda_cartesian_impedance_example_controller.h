#pragma once

#include <Eigen/Dense>
#include <reflex_controllers/base/panda_controller_base.h>

namespace panda_controllers {

struct PandaCartesianImpedanceExampleControllerPose {
  Eigen::Matrix<double, 7, 1> q_n;
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
};

struct PandaCartesianImpedanceExampleControllerParams {
  Eigen::Matrix<double, 6, 6> stiffness;
  Eigen::Matrix<double, 6, 1> damping_ratio;
  double nullspace_stiffness;
};

class PandaCartesianImpedanceExampleController :
    public virtual PandaControllerBase<PandaCartesianImpedanceExampleControllerParams,
                                       PandaCartesianImpedanceExampleControllerPose> {
 public:
  virtual ~PandaCartesianImpedanceExampleController() = default;
 private:
  void computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
      const PandaCartesianImpedanceExampleControllerPose& desired,
      const PandaCartesianImpedanceExampleControllerParams& p) override final;
  PandaCartesianImpedanceExampleControllerParams defaultParameters() override final;
  PandaCartesianImpedanceExampleControllerPose getCurrentPoseImpl() override final;
  bool hasOffsetImpl() override final;
  void resetOffset() override final;
  
  // Track start time for trajectory
  double start_time_ = -1.0;
};

}
