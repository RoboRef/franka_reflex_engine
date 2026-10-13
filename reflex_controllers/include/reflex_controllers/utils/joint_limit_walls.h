#pragma once

#include <array>
#include <vector>
#include <Eigen/Dense>

namespace panda_controllers {

class JointLimitWalls {
 public:
  JointLimitWalls() {
    moving_wall_ = {true, true, true, true, true, true, true};
    wall_scale_ = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    upper_soft_limit_ = {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};
    lower_soft_limit_ = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
  }

  void reset() {
    moving_wall_ = {true, true, true, true, true, true, true};
    wall_scale_ = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  }

  void applyWalls(const Eigen::Matrix<double, 7, 1>& q,
                  const Eigen::Matrix<double, 7, 1>& qD,
                  const Eigen::Matrix<double, 7, 1>& tau_in,
                  std::array<double, 7>& tau_out) {
    static const std::array<double, 7> pd_width =
        {{0.12, 0.09, 0.09, 0.09, 0.0349, 0.0349, 0.0349}};
    static const std::array<double, 7> d_width =
        {{0.12, 0.09, 0.09, 0.09, 0.0349, 0.0349, 0.0349}};
    static const std::array<double, 7> P =
        {{2000.0, 2000.0, 1000.0, 1000.0, 500.0, 200.0, 200.0}};
    static const std::array<double, 7> D =
        {{30.0, 30.0, 30.0, 10.0, 5.0, 5.0, 5.0}};

    for (size_t i = 0; i < 7; ++i) {
      double d_limit_max;
      double d_limit_min;
      if (moving_wall_[i]) {
        d_limit_max = upper_soft_limit_[i] - pd_width[i] - d_width[i];
        d_limit_min = lower_soft_limit_[i] + pd_width[i] + d_width[i];
        if (q[i] > d_limit_min && q[i] < d_limit_max) {
          moving_wall_[i] = false;
          wall_scale_[i] = 1.0;
        } else if (q[i] < d_limit_min && qD[i] > 0) {
          wall_scale_[i] = (d_limit_min - q[i]) / (pd_width[i] + d_width[i]);
        } else if (q[i] > d_limit_max && qD[i] < 0) {
          wall_scale_[i] = (q[i] - d_limit_max) / (pd_width[i] + d_width[i]);
        }
      }
      double pd_limit_max = upper_soft_limit_[i] - wall_scale_[i] * pd_width[i];
      double pd_limit_min = lower_soft_limit_[i] + wall_scale_[i] * pd_width[i];
      d_limit_max = upper_soft_limit_[i] - wall_scale_[i] * (pd_width[i] + d_width[i]);
      d_limit_min = lower_soft_limit_[i] + wall_scale_[i] * (pd_width[i] + d_width[i]);

      if (q[i] > pd_limit_max) {
        tau_out[i] = -D[i] * qD[i] + P[i] * (pd_limit_max - q[i]);
      } else if (q[i] < pd_limit_min) {
        tau_out[i] = -D[i] * qD[i] + P[i] * (pd_limit_min - q[i]);
      } else if (q[i] > d_limit_max) {
        tau_out[i] = -D[i] * qD[i] +
                     tau_in[i] * (q[i] - pd_limit_max) / (d_limit_max - pd_limit_max);
      } else if (q[i] < d_limit_min) {
        tau_out[i] = -D[i] * qD[i] +
                     tau_in[i] * (q[i] - pd_limit_min) / (d_limit_min - pd_limit_min);
      } else {
        tau_out[i] = tau_in[i];
      }
    }
  }

 private:
  std::array<bool, 7> moving_wall_;
  std::array<double, 7> wall_scale_;
  std::array<double, 7> lower_soft_limit_;
  std::array<double, 7> upper_soft_limit_;
};

}  // namespace panda_controllers
