/**
 * *beamsteer.cpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/beamsteer.cpp
 * @brief   Simple, deterministic beamsteering for GNSS phased array antenna
 * @date    January 2025
 * @ref     1. "Error Analysis of Carrier Phase Positioning Using Controlled Reception Pattern
 *              Antenna Arrays" (Master's thesis, Auburn University) - Josh Starling
 *          2. "A Multi-Antenna Vector Tracking Beamsteering GPS Receiver for Robust Positioning"
 *              (Master's Thesis, Auburn University) - Scott Burchfield
 * =======  ========================================================================================
 */

#include "sturdr/beamsteer.hpp"

#include <cmath>
#include <complex>
#include <navtools/constants.hpp>

namespace sturdr {

// *=== DeterministicBeam ===*
Eigen::VectorXcd Beamsteer::DeterministicBeam(
    const Eigen::Ref<const Eigen::MatrixXcd> &ant_sig, const Eigen::Ref<const Eigen::Vector3d> &u) {
  for (int i = 0; i < n_ant_; i++) {
    // calculate spatial phase based weights
    w_beam_(i) = std::exp(
        -navtools::COMPLEX_I<> * navtools::TWO_PI<> / wavelength_ * u.dot(ant_xyz_body_.col(i)));
  }
  return ant_sig * w_beam_;
}

// *=== DeterministicNull ===*
Eigen::VectorXcd Beamsteer::DeterministicNull(
    const Eigen::Ref<const Eigen::MatrixXcd> &ant_sig, const Eigen::Ref<const Eigen::Vector3d> &u) {
  for (int i = 0; i < n_ant_; i++) {
    // calculate spatial phase based weights
    w_null_(i) = std::exp(
        -navtools::COMPLEX_I<> * navtools::TWO_PI<> / wavelength_ * u.dot(ant_xyz_body_.col(i)));

    // for nulling - the sum of the N-1 secondary antennas should equal the power of the primary
    if (i > 0) {
      w_null_(i) *= null_factor_;
    }
  }
  return ant_sig * w_null_;
}

// *=== LmsSteer ===*
Eigen::VectorXcd Beamsteer::LmsSteer(
    const Eigen::Ref<const Eigen::MatrixXcd> &X, const Eigen::Ref<const Eigen::VectorXcd> &d) {
  Eigen::VectorXcd y(Eigen::VectorXcd::Zero(X.cols()));

  // calculate current best estimate of signal
  y = X * W_.transpose();

  // update weights for better estimate next iteration
  W_ += 2.0 * mu_ * (d - y).transpose() * X;
  W_ /= W_.norm();

  return y;
}

// *=== LmsNull ===*
Eigen::VectorXcd Beamsteer::LmsNull(const Eigen::Ref<const Eigen::MatrixXcd> &X) {
  Eigen::VectorXcd y(Eigen::VectorXcd::Zero(X.cols()));

  // calculate current best estimate of signal
  y = X * W_.transpose();

  // update weights for better estimate next iteration
  W_ -= 2.0 * mu_ * y.transpose() * X;
  W_ /= W_.norm();

  return y;
}

}  // namespace sturdr