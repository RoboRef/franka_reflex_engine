#pragma once

#include <Eigen/Dense>
#include <reflex_controllers/base/panda_controller_base.h>
#include <reflex_controllers/utils/joint_limit_walls.h>

namespace panda_controllers {

struct FrankaCartesianRetractControllerPose {
  Eigen::Matrix<double, 7, 1> q_n;
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
};

struct FrankaCartesianRetractControllerParams {
  double retraction_distance{0.05}; // meters (5 cm)
};

class FrankaCartesianRetractController :
    public virtual PandaControllerBase<FrankaCartesianRetractControllerParams,
                                       FrankaCartesianRetractControllerPose> {
 public:
  virtual ~FrankaCartesianRetractController() = default;

 private:
  void computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
                      const FrankaCartesianRetractControllerPose& desired,
                      const FrankaCartesianRetractControllerParams& p) override;
  FrankaCartesianRetractControllerParams defaultParameters() override;
  FrankaCartesianRetractControllerPose getCurrentPoseImpl() override;
  bool hasOffsetImpl() override;
  void resetOffset() override;
  void startImpl() override;

  Eigen::Vector3d p_start_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d p_goal_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q_orientation_{Eigen::Quaterniond::Identity()};
  Eigen::Matrix<double, 7, 1> q_start_{Eigen::Matrix<double, 7, 1>::Zero()};
  JointLimitWalls walls_;
};

}  // namespace panda_controllers
