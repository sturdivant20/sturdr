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

#include "sturdr/nav/estimation.hpp"

#include <spdlog/spdlog.h>

#include <navtools/constants.hpp>

#include "sturdr/dsp/discriminator.hpp"

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
  // std::cout << "dr = " << dr.transpose() << "\n";
  // std::cout << "dv = " << dv.transpose() << "\n";
  // std::cout << "dv * r^2 = " << (dv * r2).transpose() << "\n";
  // std::cout << "dv @ dr = " << dv.dot(dr) << "\n";
  // std::cout << "-dr * (dv @ dr) = " << (-dr * dv.dot(dr)).transpose() << "\n";
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
    spdlog::default_logger()->error("estimation.cpp NavKF::Dynamics failed! Error -> {}", e.what());
    return false;
  }
}
bool NavKF::Dynamics(const double &T) {
  make_F(T);
  make_Q(T);
  return Dynamics();
}

// *=== Measurement ===*
template <int N>
bool NavKF::Measurement(
    const Eigen::Matrix<double, N, 3> &sv_pos,
    const Eigen::Matrix<double, N, 3> &sv_vel,
    const Eigen::Vector<double, N> &psr,
    const Eigen::Vector<double, N> &psrdot,
    const Eigen::Vector<double, N> &cno,
    const double &beta,
    const double &lambda,
    const double &T) {
  try {
    // Initialize
    const int M = 2 * N;
    const Eigen::Vector<double, N> one = Eigen::Vector<double, N>::One();
    Eigen::Matrix<double, M, 8> H = Eigen::Matrix<double, M, 8>::Zero();
    H.col(6).segment<N>(0) = one;  // H.block<N, 1>(0, 6) = one;
    H.col(7).segment<N>(N) = one;  // H.block<N, 1>(N, 7) = one;
    Eigen::Vector<double, M> dz;

    double beta2 = beta * beta;
    double lambda2 = lambda * lambda;
    Eigen::Matrix<double, M, M> R = Eigen::Matrix<double, M, M>::Zero();
    for (int i = 0; i < M; i++) {
      R(i, i) = beta2 * DllVariance(cno(i), T);
      R(N + i, N + i) = lambda2 * FllVariance(cno(i), T);
    }

    // Innovation
    Eigen::Vector3d pos = x_.segment(0, 3);
    Eigen::Vector3d vel = x_.segment(3, 3);
    Eigen::Vector3d u, udot;
    double pred_psr, pred_psrdot;
    for (int i = 0; i < N; i++) {
      RangeAndRate(
          pos, vel, x_(6), x_(7), sv_pos.col(i), sv_vel.col(i), u, udot, pred_psr, pred_psrdot);

      // update the measurement matrix and innovation
      H.row(i).segment<3>(0) = u.transpose();         // H.block<1,3>(i,0)=u.transpose();
      H.row(N + i).segment<3>(0) = udot.transpose();  // H.block<1,3>(N+i,0)=udot.transpose();
      H.row(N + i).segment<3>(3) = udot.transpose();  // H.block<1,3>(N+i,3)=u.transpose();
      dz(i) = psr(i) - pred_psr;
      dz(N + i) = psrdot(i) - pred_psrdot;
    }

    // Innovation filter
    Eigen::Matrix<double, M, M> S = H * P_ * H.transpose() + R;
    Eigen::Vector<double, M> norm_dz = (dz.array() / S.diagonal().array().sqrt()).abs();
    Eigen::Array<bool, M, 1> mask = norm_dz.array() < inn_std_;
    const int M2 = mask.count();
    if (M2 < M) {
      // an innovation got flagged
      Eigen::Matrix<double, M2, 8> _H = Eigen::Matrix<double, M2, 8>::Zero();
      Eigen::Matrix<double, M2, M2> _R = Eigen::Matrix<double, M2, M2>::Zero();
      Eigen::Vector<double, M2> _dz;
      int j = 0;
      for (int i = 0; i < M; i++) {
        if (mask(i)) {
          _H.row(j) = H.row(i);
          _R(j, j) = R(i, i);
          _dz(j) = dz(i);
          j++;
        }
      }
      kalman_update(_H, _R, _dz);
    } else {
      // proceed as normal
      kalman_update(H, R, dz);
    }

    return true;
  } catch (std::exception &e) {
    spdlog::default_logger()->error(
        "estimation.cpp NavKF::Measurement failed! Error -> {}", e.what());
    return false;
  }
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