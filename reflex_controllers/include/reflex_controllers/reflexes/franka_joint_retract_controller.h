#pragma once

#include <Eigen/Dense>
#include <reflex_controllers/base/panda_controller_base.h>
#include <reflex_controllers/utils/joint_limit_walls.h>

namespace panda_controllers {

struct FrankaJointRetractControllerPose {
  Eigen::Matrix<double, 7, 1> q;
  Eigen::Matrix<double, 7, 1> qD;
};

struct FrankaJointRetractControllerParams {
  double max_retraction{0.15}; // rad max retraction norm
};

class FrankaJointRetractController :
    public virtual PandaControllerBase<FrankaJointRetractControllerParams,
                                       FrankaJointRetractControllerPose> {
 public:
  virtual ~FrankaJointRetractController() = default;

 private:
  void computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
                      const FrankaJointRetractControllerPose& desired,
                      const FrankaJointRetractControllerParams& p) override;
  FrankaJointRetractControllerParams defaultParameters() override;
  FrankaJointRetractControllerPose getCurrentPoseImpl() override;
  bool hasOffsetImpl() override;
  void resetOffset() override;
  void startImpl() override;

  Eigen::Matrix<double, 7, 1> q_start_;
  Eigen::Matrix<double, 7, 1> q_goal_;
  JointLimitWalls walls_;
};

}  // namespace panda_controllers
