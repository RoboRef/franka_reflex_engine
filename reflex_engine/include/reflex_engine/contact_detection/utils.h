#pragma once

#include <Eigen/Dense>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <franka/robot_state.h>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace reflex_engine {

inline bool getTransform(rclcpp_lifecycle::LifecycleNode::SharedPtr node, const std::string& from, const std::string& to,
  Eigen::Vector3d& r_t_f, Eigen::Matrix3d& t_R_f) {

  // auto node = rclcpp::Node::make_shared("transform_listener_node");
  tf2_ros::Buffer tfBuffer(node->get_clock());
  tf2_ros::TransformListener tfListener(tfBuffer);
  std::string error;
  if (!tfBuffer.canTransform(to, from, tf2::TimePointZero, tf2::durationFromSec(3.0),
    &error)) {
    RCLCPP_ERROR(node->get_logger(), "Could not transform from %s to %s. Error: %s",
    from.c_str(), to.c_str(), error.c_str());
    return false;
  }
  geometry_msgs::msg::TransformStamped transform;
  transform = tfBuffer.lookupTransform(to, from, tf2::TimePointZero,
                    tf2::durationFromSec(3.0));
  r_t_f = Eigen::Vector3d(transform.transform.translation.x,
                          transform.transform.translation.y,
                          transform.transform.translation.z);
  Eigen::Quaterniond tmp( transform.transform.rotation.w,
                          transform.transform.rotation.x,
                          transform.transform.rotation.y,
                          transform.transform.rotation.z);
  t_R_f = tmp.toRotationMatrix();
  return true;
}

// inline bool getTransform(const std::string& from, const std::string& to,
//                          Eigen::Vector3d& r_t_f, Eigen::Matrix3d& t_R_f) {
//   tf2_ros::Buffer tfBuffer;
//   tf2_ros::TransformListener tfListener(tfBuffer);
//   std::string error;
//   if (!tfBuffer.canTransform(to, from, ros::Time(0), ros::Duration(3),
//       &error)) {
//     ROS_ERROR("Could not transform from %s to %s. Error: %s",
//               from.c_str(), to.c_str(), error.c_str());
//     return false;
//   }
//   geometry_msgs::msg::TransformStamped transform;
//   transform = tfBuffer.lookupTransform(to, from, ros::Time(0),
//                                        ros::Duration(3));
//   r_t_f = Eigen::Vector3d(transform.transform.translation.x,
//                           transform.transform.translation.y,
//                           transform.transform.translation.z);
//   Eigen::Quaterniond tmp(transform.transform.rotation.w,
//                          transform.transform.rotation.x,
//                          transform.transform.rotation.y,
//                          transform.transform.rotation.z);
//   t_R_f = tmp.toRotationMatrix();
//   return true;
// }

template <typename Min, typename Max, typename Checked>
inline bool checkThresholds(const Eigen::DenseBase<Min>& min,
                            const Eigen::DenseBase<Max>& max,
                            const Eigen::DenseBase<Checked>& checked) {
  for (int i=0; i<checked.size(); ++i) {
    if (checked[i] > max[i] || checked[i] < min[i])
      return true;
  }
  return false;
}

inline Eigen::Matrix<double, 6, 1> transformWrenchToO(
    const franka::RobotState& state) {
  Eigen::Vector3d O_f_ext_O;
  Eigen::Vector3d O_m_ext_O;
  Eigen::Matrix4d O_T_EE(state.O_T_EE.data());
  Eigen::Matrix4d EE_T_K(state.EE_T_K.data());
  Eigen::Matrix4d O_T_K = O_T_EE * EE_T_K;
  Eigen::Vector3d r_OK = O_T_K.topRows(3).col(3);
  for (int i=0; i<3; ++i) {
    O_f_ext_O(i) = state.O_F_ext_hat_K[i];
    O_m_ext_O(i) = state.O_F_ext_hat_K[i+3];
  }
  Eigen::Matrix<double, 6, 1> O_F_ext_K;
  O_F_ext_K.head(3) = O_f_ext_O;
  O_F_ext_K.tail(3) = O_m_ext_O - r_OK.cross(O_f_ext_O);
  return O_F_ext_K;
}

inline Eigen::Matrix<double, 6, 1> getContactWrenchB(
    const Eigen::Matrix<double, 6, 7>& J, const franka::RobotState& state,
    const Eigen::Matrix3d& B_R_O) {
  Eigen::Matrix<double, 6, 1> O_F_ext_K;
  if (Eigen::Matrix<double, 6 ,1>::Map(state.K_F_ext_hat_K.data()).norm()==0) {
    Eigen::JacobiSVD<Eigen::Matrix<double, 7, 6>> svd(J.transpose(),
        Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix<double, 7, 6> sv_inv = Eigen::Matrix<double, 7, 6>::Zero();
    for (int i=0;i<6;++i) {
      if (svd.singularValues()[i] < 0.08) {
        sv_inv(i,i) = 1/0.08;
      } else {
        sv_inv(i,i) = 1/svd.singularValues()[i];
      }
    }
    Eigen::Matrix<double, 6, 7> pinv =
        svd.matrixV() * sv_inv.transpose() * svd.matrixU().adjoint();
    O_F_ext_K = pinv *
        Eigen::Matrix<double, 7, 1>::Map(state.tau_ext_hat_filtered.data());
  } else {
    O_F_ext_K = transformWrenchToO(state);
  }
  Eigen::Matrix<double, 6, 1> B_F_ext_K;
  B_F_ext_K.head(3) = B_R_O * O_F_ext_K.head(3);
  B_F_ext_K.tail(3) = B_R_O * O_F_ext_K.tail(3);
  return B_F_ext_K;
}

}
