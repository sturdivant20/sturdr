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

#pragma once

#ifndef STURDR_ESTIMATION_HPP
#define STURDR_ESTIMATION_HPP

#include <spdlog/spdlog.h>

#include <Eigen/Dense>
#include <navtools/constants.hpp>

#include "sturdr/dsp/discriminator.hpp"
#include "sturdr/nav/clock.hpp"

namespace sturdr {

/**
 * *=== RangeAndRate ===*
 */
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
    double &pred_psrdot);

/**
 * *=== LeastSquares ===*
 * @brief Least Squares solver for GNSS position, velocity, and timing terms
 * @param x       Initial state estimate
 * @param P       Initial covariance estimate
 * @param sv_pos  Satellite ECEF positions [m]
 * @param sv_vel  Satellite ECEF velocities [m/s]
 * @param psr     Pseudorange measurements [m]
 * @param psrdot  Pseudorange-rate measurements [m/s]
 * @param cno     Carrier-to-noise density ratio measurements (not dB-Hz)
 * @param beta    Signals chip width [m/chip]
 * @param lambda  Signals wavelength [m/rad]
 * @param T       Measurement Integration period [s]
 */
template <int N>
bool LeastSquares(
    Eigen::Vector<double, 8> &x,
    Eigen::Matrix<double, 8, 8> &P,
    const Eigen::Matrix<double, 3, N> &sv_pos,
    const Eigen::Matrix<double, 3, N> &sv_vel,
    const Eigen::Vector<double, N> &psr,
    const Eigen::Vector<double, N> &psrdot,
    const Eigen::Vector<double, N> &cno,
    const double &beta,
    const double &lambda,
    const double &T) {
  try {
    // Initialize
    const int M = 2 * N;
    const Eigen::Vector<double, N> one = Eigen::Vector<double, N>::Ones();
    Eigen::Matrix<double, M, 8> H = Eigen::Matrix<double, M, 8>::Zero();
    H.block(0, 6, N, 1) = one;
    H.block(N, 7, N, 1) = one;
    Eigen::Vector<double, M> dz;

    // Measurement covariance weighting
    double beta2 = beta * beta;
    double lambda2 = lambda * lambda / navtools::PI_SQU<double>;
    Eigen::Matrix<double, M, M> R = Eigen::Matrix<double, M, M>::Zero();
    for (int i = 0; i < N; i++) {
      R(i, i) = 1.0 / (beta2 * DllVariance(cno(i), T));
      R(N + i, N + i) = 1.0 / (lambda2 * FllVariance(cno(i), T));
    }

    // Recursive estimation
    Eigen::Vector<double, 8> dx;
    Eigen::Vector3d u, udot;
    double pred_psr, pred_psrdot;
    for (int k = 0; k < 20; k++) {
      // update position and velocity estimates

      for (int i = 0; i < N; i++) {
        RangeAndRate(
            x.segment(0, 3),
            x.segment(3, 3),
            x(6),
            x(7),
            sv_pos.col(i),
            sv_vel.col(i),
            u,
            udot,
            pred_psr,
            pred_psrdot);

        // update the measurement matrix and innovation
        H.block(i, 0, 1, 3) = u.transpose();
        H.block(N + i, 0, 1, 3) = udot.transpose();
        H.block(N + i, 3, 1, 3) = u.transpose();
        dz(i) = psr(i) - pred_psr;
        dz(N + i) = psrdot(i) - pred_psrdot;
      }

      // Weighted least squares
      P = (H.transpose() * R * H).inverse();
      dx = P * H.transpose() * R * dz;
      x += dx;
      if (dx.segment(0, 3).norm() < 1e-6) {
        break;
      }
    }
    return true;
  } catch (std::exception &e) {
    spdlog::get("sturdr-console")
        ->error("estimation.cpp LeastSquares failed! Error -> {}", e.what());
    return false;
  }
};

//! ------------------------------------------------------------------------------------------------

class NavKF {
 public:
  /**
   * *=== NavKF ===*
   * @brief Constructor
   * @param x           Initial state vector (x,y,z,vx,vy,vz,c*t,c*tdot)
   * @param P           Initial state covariance matrix
   * @param T           Expected dynamics integration period
   * @param process_std Dynamics process noise standard deviation
   * @param innov_std   Measurement innovation noise allowance threshold
   * @param clock_model Navigation clock model
   */
  NavKF(
      const Eigen::Vector<double, 8> &x,
      const Eigen::Matrix<double, 8, 8> &P,
      const double &T,
      const double &process_std,
      const double &innov_std,
      const std::string &clock_model);

  /**
   * *=== ~NavKF ===*
   * @brief Destructor
   */
  ~NavKF();

  /**
   * *=== Dynamics ===*
   * @brief Run the dynamics propagation
   * @param T   Integration period [s]
   * @return True|False based on success
   */
  bool Dynamics();
  bool Dynamics(const double &T);

  /**
   * *=== Measurement ===*
   * @brief Run the measurement correction
   * @param sv_pos  Satellite ECEF positions [m]
   * @param sv_vel  Satellite ECEF velocities [m/s]
   * @param psr     Pseudorange measurements [m]
   * @param psrdot  Pseudorange-rate measurements [m/s]
   * @param cno     Carrier-to-noise density ratio measurements (not dB-Hz)
   * @param beta    Signals chip width [m/chip]
   * @param lambda  Signals wavelength [m/rad]
   * @param T       Measurement Integration period [s]
   * @param
   */
  template <int N>
  bool Measurement(
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
      const Eigen::Vector<double, N> one = Eigen::Vector<double, N>::Ones();
      Eigen::Matrix<double, M, 8> H = Eigen::Matrix<double, M, 8>::Zero();
      H.block(0, 6, N, 1) = one;
      H.block(N, 7, N, 1) = one;
      Eigen::Vector<double, M> dz;

      // Measurement covariance weighting
      double beta2 = beta * beta;
      double lambda2 = lambda * lambda / navtools::PI_SQU<double>;
      Eigen::Matrix<double, M, M> R = Eigen::Matrix<double, M, M>::Zero();
      for (int i = 0; i < N; i++) {
        R(i, i) = beta2 * DllVariance(cno(i), T);
        R(N + i, N + i) = lambda2 * FllVariance(cno(i), T);
      }

      // Innovation
      Eigen::Vector<double, 8> dx;
      Eigen::Vector3d u, udot;
      double pred_psr, pred_psrdot;
      for (int i = 0; i < N; i++) {
        RangeAndRate(
            x_.segment(0, 3),
            x_.segment(3, 3),
            x_(6),
            x_(7),
            sv_pos.col(i),
            sv_vel.col(i),
            u,
            udot,
            pred_psr,
            pred_psrdot);

        // update the measurement matrix and innovation
        H.block(i, 0, 1, 3) = u.transpose();
        H.block(N + i, 0, 1, 3) = udot.transpose();
        H.block(N + i, 3, 1, 3) = u.transpose();
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
      spdlog::get("sturdr-console")
          ->error("estimation.cpp NavKF::Measurement failed! Error -> {}", e.what());
      return false;
    }
  }

 private:
  Eigen::Vector<double, 8> x_;
  Eigen::Matrix<double, 8, 8> P_;
  Eigen::Matrix<double, 8, 8> F_;
  Eigen::Matrix<double, 8, 8> Q_;
  NavigationClock clk_;
  double dyn_var_;
  double inn_std_;

  void make_F(const double &T);
  void make_Q(const double &T);
  template <int M>
  void kalman_update(
      const Eigen::Matrix<double, M, 8> &H,
      const Eigen::Matrix<double, M, M> &R,
      const Eigen::Vector<double, 8> &dz);
};

}  // namespace sturdr

#endif