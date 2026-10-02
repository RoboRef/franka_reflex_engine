#pragma once

#include <Eigen/Dense>
#include <reflex_controllers/base/panda_controller_base.h>

namespace panda_controllers {

struct ComlessMoveDownControllerPose {
  Eigen::Matrix<double, 7, 1> q_n;
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
};

struct ComlessMoveDownControllerParams {
  Eigen::Matrix<double, 6, 6> stiffness;
  Eigen::Matrix<double, 6, 1> damping_ratio;
  double nullspace_stiffness;
  
  // Custom parameters for the move down trajectory
  double max_velocity;
  double acceleration;
  double distance;
};

class ComlessMoveDownController :
    public virtual PandaControllerBase<ComlessMoveDownControllerParams,
                                       ComlessMoveDownControllerPose> {
 public:
  virtual ~ComlessMoveDownController() = default;
 private:
  void startImpl() override final;
  void computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
      const ComlessMoveDownControllerPose& desired,
      const ComlessMoveDownControllerParams& p) override final;
  ComlessMoveDownControllerParams defaultParameters() override final;
  ComlessMoveDownControllerPose getCurrentPoseImpl() override final;
  bool hasOffsetImpl() override final;
  void resetOffset() override final;

  // Trajectory state
  bool initialized_start_pose_ = false;
  double time_traj_ = 0.0;
  Eigen::Vector3d initial_position_;
  Eigen::Quaterniond initial_orientation_;
};

}
