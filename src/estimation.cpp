/**
 * *estimation.cpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/nav/estimation.cpp
 * @brief   Satellite navigation techniques.
 * @date    December 2024
 * @author  Daniel Sturdivant <sturdivant20@gmail.com>
 * @ref     1. "Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems", 2nd
 *              Edition, 2013 - Groves
 *          2. "Global Positioning System: Signals, Measurements, and Performance", 2nd Edition,
 *              2006 - Misra & Enge
 * =======  ========================================================================================
 */

#include "sturdr/estimation.hpp"

#include <spdlog/spdlog.h>

#include <navtools/constants.hpp>

namespace sturdr {

inline static const Eigen::Matrix<double, 8, 8> I8 = Eigen::Matrix<double, 8, 8>::Identity();

void RangeAndRate(
    const Eigen::Ref<const Eigen::Vector3d> &pos,
    const Eigen::Ref<const Eigen::Vector3d> &vel,
    const double &cb,
    const double &cd,
    const Eigen::Ref<const Eigen::Vector3d> &sv_pos,
    const Eigen::Ref<const Eigen::Vector3d> &sv_vel,
    Eigen::Ref<Eigen::Vector3d> u,
    Eigen::Ref<Eigen::Vector3d> udot,
    double &pred_psr,
    double &pred_psrdot) {
  // predict approximate range and account for earth's rotation (Groves 8.34)
  Eigen::Vector3d dr = pos - sv_pos;
  double r = dr.norm();
  double wtau = navtools::WGS84_OMEGA<double> * r / navtools::LIGHT_SPEED<double>;
  double swtau = std::sin(wtau);
  double cwtau = std::cos(wtau);
  Eigen::Matrix3d C_i_e{{cwtau, swtau, 0.0}, {-swtau, cwtau, 0.0}, {0.0, 0.0, 1.0}};

  // predict pseudorange (Groves 8.35, 9.165)
  dr = pos - C_i_e * sv_pos;
  r = dr.norm();
  u = dr.array() / r;
  pred_psr = r + cb;

  // predict pseudorange-rate (Groves 8.44, 9.165)
  double r2 = r * r;
  Eigen::Vector3d dv = (vel + navtools::WGS84_OMEGA_SKEW<double> * pos) -
                       C_i_e * (sv_vel + navtools::WGS84_OMEGA_SKEW<double> * sv_pos);
  udot = (dv * r2 - dr * dv.dot(dr)) / (r2 * r);
  pred_psrdot = u.dot(dv) + cd;
}

//! ------------------------------------------------------------------------------------------------

//* === NavKF ===
NavKF::NavKF(
    const Eigen::Vector<double, 8> &x,
    const Eigen::Matrix<double, 8, 8> &P,
    const double &T,
    const double &process_std,
    const double &innov_std,
    const std::string &clock_model)
    : x_{x},
      P_{P},
      clk_{GetNavClock(clock_model)},
      dyn_var_{0.5 * process_std * process_std},
      inn_std_{innov_std} {
  // Initialize dynamics matrices
  F_ = Eigen::Matrix<double, 8, 8>::Identity();
  Q_ = Eigen::Matrix<double, 8, 8>::Zero();
  make_F(T);
  make_Q(T);
}

// *=== ~NavKF ===*
NavKF::~NavKF() {
}

// *=== Dynamics ===*
bool NavKF::Dynamics() {
  try {
    x_ = F_ * x_;
    P_ = F_ * (P_ + Q_) * F_.transpose() + Q_;
    return true;
  } catch (std::exception &e) {
    spdlog::get("sturdr-console")
        ->error("estimation.cpp NavKF::Dynamics failed! Error -> {}", e.what());
    return false;
  }
}
bool NavKF::Dynamics(const double &T) {
  make_F(T);
  make_Q(T);
  return Dynamics();
}

//* === make_F ===*
void NavKF::make_F(const double &T) {
  F_.block<3, 3>(0, 3).diagonal() << T, T, T;
  F_(6, 7) = T;
}

//* === make_Q ===*
void NavKF::make_Q(const double &T) {
  double LS2 = 0.5 * navtools::LIGHT_SPEED<double> * navtools::LIGHT_SPEED<double>;
  double T2 = T * T;
  double T3 = T * T2;
  double q_bb = LS2 * ((clk_.h0 * T / 2.0) + (2.0 * clk_.h1 * T2) +
                       (2.0 / 3.0 * navtools::PI_SQU<double> * clk_.h2 * T3));
  double q_bd = LS2 * ((2.0 * clk_.h1 * T) + (navtools::PI_SQU<double> * clk_.h2 * T2));
  double q_dd = LS2 * ((clk_.h0 / 2.0 / T) + (2.0 * clk_.h1) +
                       (8.0 / 3.0 * navtools::PI_SQU<double> * clk_.h2 * T));
  double q_pp = dyn_var_ * T3 / 3.0;
  double q_pv = dyn_var_ * T2 / 2.0;
  double q_vv = dyn_var_ * T;

  Q_.block<3, 3>(0, 0).diagonal() << q_pp, q_pp, q_pp;
  Q_.block<3, 3>(0, 3).diagonal() << q_pv, q_pv, q_pv;
  Q_.block<3, 3>(3, 0).diagonal() << q_pv, q_pv, q_pv;
  Q_.block<3, 3>(3, 3).diagonal() << q_vv, q_vv, q_vv;
  Q_(6, 6) = q_bb;
  Q_(6, 7) = q_bd;
  Q_(7, 6) = q_bd;
  Q_(7, 7) = q_dd;
}

// *=== kalman_update ===*
template <int M>
void NavKF::kalman_update(
    const Eigen::Matrix<double, M, 8> &H,
    const Eigen::Matrix<double, M, M> &R,
    const Eigen::Vector<double, 8> &dz) {
  Eigen::Matrix<double, 8, M> PHt = P_ * H.transpose();
  Eigen::Matrix<double, 8, M> K = PHt * (H * PHt + R).inverse();
  P_ = (I8 - K * H) * P_;
  x_ += K * dz;
}

}  // namespace sturdr