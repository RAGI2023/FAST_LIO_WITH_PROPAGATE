#pragma once

#include <Eigen/src/Geometry/Quaternion.h>

#include "use-ikfom.hpp"

class ImuPredicter {
 public:
  ImuPredicter() {}
  ImuPredicter(const Eigen::Vector3d& g) { this->g = g; }
  void fastPredictIMU(double t, Eigen::Vector3d linear_acceleration,
                      Eigen::Vector3d angular_velocity) {
    linear_acceleration =
        changeImuDataUnit(linear_acceleration);  // mid360 转换单位
    if (!latest_time) {
      // 初始化
      latest_time = t;
      latest_acc_0 = linear_acceleration;
      latest_gyr_0 = angular_velocity;
      return;
    }
    double dt = t - latest_time;
    latest_time = t;
    Eigen::Vector3d un_acc_0 = latest_Q * (latest_acc_0 - latest_Ba) - g;
    Eigen::Vector3d un_gyr =
        0.5 * (latest_gyr_0 + angular_velocity) - latest_Bg;
    latest_Q = latest_Q * deltaQ(un_gyr * dt);
    Eigen::Vector3d un_acc_1 = latest_Q * (linear_acceleration - latest_Ba) - g;
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    latest_P = latest_P + dt * latest_V + 0.5 * dt * dt * un_acc;
    latest_V = latest_V + dt * un_acc;
    latest_acc_0 = linear_acceleration;
    latest_gyr_0 = angular_velocity;
  }
  double latest_time = 0.0;
  Eigen::Vector3d g;
  Eigen::Quaterniond latest_Q;
  Eigen::Vector3d latest_acc_0;
  Eigen::Vector3d latest_Ba;
  Eigen::Vector3d latest_gyr_0;
  Eigen::Vector3d latest_Bg;
  Eigen::Vector3d latest_P, latest_V;

  void set_g(const Eigen::Vector3d& g) { this->g = g; }

  void set_bias(const state_ikfom& ikfom) {
    latest_Ba = ikfom.ba;
    latest_Bg = ikfom.bg;
  }

  void set_state(const state_ikfom& ikfom) {
    latest_Q = ikfom.rot;
    latest_P = ikfom.pos;
    latest_V = ikfom.vel;
  }

  void get_state(Eigen::Vector3d& position, Eigen::Quaterniond& q,
                 Eigen::Vector3d& vel) {
    position = latest_P;
    q = latest_Q;
    vel = latest_V;
  }

 private:
  static Eigen::Vector3d changeImuDataUnit(
      const Eigen::Vector3d linear_acceleration) {
    return linear_acceleration * -9.81;
  }
  static Eigen::Quaterniond deltaQ(const Eigen::Vector3d& theta) {
    Eigen::Quaterniond dq;
    Eigen::Vector3d half_theta = theta / 2.0;

    dq.w() = 1.0;
    dq.x() = half_theta.x();
    dq.y() = half_theta.y();
    dq.z() = half_theta.z();
    return dq;
  }
};