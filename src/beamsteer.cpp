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
#include <iostream>
#include <navtools/constants.hpp>

namespace sturdr {

// *=== DeterministicBeam ===*
void DeterministicBeam(
    const Eigen::Ref<const Eigen::MatrixXd> &ant_xyz,
    const Eigen::Ref<const Eigen::Vector3d> &u,
    const Eigen::Ref<const Eigen::MatrixXcd> &rfdata,
    const bool code[1023],
    double &rem_code_phase,
    double &code_freq,
    double &rem_carr_phase,
    double &carr_freq,
    double &carr_jit,
    double &samp_freq,
    uint64_t &half_samp,
    uint64_t &samp_remaining,
    double &t_space,
    std::complex<double> &E,
    std::complex<double> &P1,
    std::complex<double> &P2,
    std::complex<double> &L) {
  // init phase increments
  double d_code = code_freq / samp_freq;
  double d_carr = (carr_freq + 0.5 * carr_jit / samp_freq) / samp_freq;

  // calculate spatial phase based weights
  int n_ant = ant_xyz.cols();
  Eigen::VectorXcd W(n_ant);
  for (int i = 0; i < n_ant; i++) {
    W(i) = std::exp(-navtools::COMPLEX_I<> * u.dot(ant_xyz.col(i)));
  }

  // loop through number of samples
  int n_samp = rfdata.rows();
  std::complex<double> v_carr;
  std::complex<double> sample;
  double v_code;
  for (int i = 0; i < n_samp; i++) {
    // combine 'n_ant' signals together with deterministic weights
    sample = rfdata.row(i) * W;

    // carrier replica value
    v_carr = std::exp(-navtools::COMPLEX_I<> * rem_carr_phase) * sample;

    // early
    // v_code = code[static_cast<int>(std::fmod(rem_code_phase + t_space, 1023.0))] ? 1.0 : -1.0;
    v_code = code[static_cast<int>(std::round(rem_code_phase + t_space)) % 1023] ? 1.0 : -1.0;
    E += (v_code * v_carr);

    // late
    // v_code = code[static_cast<int>(std::fmod(rem_code_phase - t_space, 1023.0))] ? 1.0 : -1.0;
    v_code = code[static_cast<int>(std::round(rem_code_phase - t_space)) % 1023] ? 1.0 : -1.0;
    L += (v_code * v_carr);

    // prompt
    // v_code = code[static_cast<int>(std::fmod(rem_code_phase, 1023.0))] ? 1.0 : -1.0;
    v_code = code[static_cast<int>(std::round(rem_code_phase)) % 1023] ? 1.0 : -1.0;
    if (samp_remaining > half_samp) {
      P1 += (v_code * v_carr);
    } else {
      P2 += (v_code * v_carr);
    }

    // increment
    rem_code_phase += d_code;
    rem_carr_phase += d_carr;
    samp_remaining--;
  }
}

// *=== DeterministicNull ===*
void DeterministicNull(
    const Eigen::Ref<const Eigen::MatrixXd> &ant_xyz,
    const Eigen::Ref<const Eigen::Vector3d> &u,
    const Eigen::Ref<const Eigen::MatrixXcd> &rfdata,
    const bool code[1023],
    double &rem_code_phase,
    double &code_freq,
    double &rem_carr_phase,
    double &carr_freq,
    double &carr_jit,
    double &samp_freq,
    uint64_t &half_samp,
    uint64_t &samp_remaining,
    double &t_space,
    std::complex<double> &E,
    std::complex<double> &P1,
    std::complex<double> &P2,
    std::complex<double> &L) {
  // init phase increments
  double d_code = code_freq / samp_freq;
  double d_carr = (carr_freq + 0.5 * carr_jit / samp_freq) / samp_freq;

  // calculate spatial phase based weights
  int n_ant = ant_xyz.cols();
  double null_factor = 1.0 / static_cast<double>(n_ant - 1);
  Eigen::VectorXcd W(n_ant);
  for (int i = 0; i < n_ant; i++) {
    // calculate spatial phase based weights
    W(i) = std::exp(-navtools::COMPLEX_I<> * u.dot(ant_xyz.col(i)));

    // for nulling - the sum of the N-1 secondary antennas should equal the power of the primary
    if (i > 0) {
      W(i) *= null_factor;
    }
  }

  // loop through number of samples
  int n_samp = rfdata.rows();
  std::complex<double> v_carr;
  std::complex<double> sample;
  double v_code;
  for (int i = 0; i < n_samp; i++) {
    // combine 'n_ant' signals together with deterministic weights
    sample = rfdata.row(i) * W;

    // carrier replica value
    v_carr = std::exp(-navtools::COMPLEX_I<> * rem_carr_phase) * sample;

    // early
    // v_code = code[static_cast<int>(std::fmod(rem_code_phase + t_space, 1023.0))] ? 1.0 : -1.0;
    v_code = code[static_cast<int>(std::round(rem_code_phase + t_space)) % 1023] ? 1.0 : -1.0;
    E += (v_code * v_carr);

    // late
    // v_code = code[static_cast<int>(std::fmod(rem_code_phase - t_space, 1023.0))] ? 1.0 : -1.0;
    v_code = code[static_cast<int>(std::round(rem_code_phase - t_space)) % 1023] ? 1.0 : -1.0;
    L += (v_code * v_carr);

    // prompt
    // v_code = code[static_cast<int>(std::fmod(rem_code_phase, 1023.0))] ? 1.0 : -1.0;
    v_code = code[static_cast<int>(std::round(rem_code_phase)) % 1023] ? 1.0 : -1.0;
    if (samp_remaining > half_samp) {
      P1 += (v_code * v_carr);
    } else {
      P2 += (v_code * v_carr);
    }

    // increment
    rem_code_phase += d_code;
    rem_carr_phase += d_carr;
    samp_remaining--;
  }
}

// *=== LmsBeam ===*
void LmsBeam(
    const double &mu,
    Eigen::Ref<Eigen::VectorXcd> W,
    const Eigen::Ref<const Eigen::MatrixXcd> &rfdata,
    const bool code[1023],
    double &rem_code_phase,
    double &code_freq,
    double &rem_carr_phase,
    double &carr_freq,
    double &carr_jit,
    double &samp_freq,
    uint64_t &half_samp,
    uint64_t &samp_remaining,
    double &t_space,
    std::complex<double> &E,
    std::complex<double> &P1,
    std::complex<double> &P2,
    std::complex<double> &L) {
  // init phase increments
  double d_code = code_freq / samp_freq;
  double d_carr = (carr_freq + 0.5 * carr_jit / samp_freq) / samp_freq;

  //! W  is 1 x N
  //! X  is S x N
  //! y  is S x 1
  //! d  is S x 1 (if nulling d = 0)
  //! mu is 1 x 1

  //! y = X * W.transpose()
  //! W = W + 2 * mu * (d - y).transpose() * X
  std::cout << "W: " << W(0) << ", " << W(1) << ", " << W(2) << ", " << W(3) << "\n";

  // loop through number of samples
  int n_samp = rfdata.rows();
  Eigen::VectorXcd X(W.size()), S(W.size());
  Eigen::MatrixXcd Phi(W.size(), W.size());
  std::complex<double> v_carr, y, P;
  double v_code;
  for (int i = 0; i < n_samp; i++) {
    // carrier replica
    v_carr = std::exp(-navtools::COMPLEX_I<> * rem_carr_phase);

    // sample vector
    X = rfdata.row(i) * v_carr;
    y = X.sum();

    // sample covariance
    Phi = X.conjugate() * X.transpose();

    // prompt
    // v_code = code[static_cast<int>(std::fmod(rem_code_phase, 1023.0))] ? 1.0 : -1.0;
    v_code = code[static_cast<int>(std::round(rem_code_phase)) % 1023] ? 1.0 : -1.0;
    P = v_code * y;
    if (samp_remaining > half_samp) {
      P1 += P;
    } else {
      P2 += P;
    }

    // reference correlation vector
    if (P.real() > 0.0) {
      S = X.conjugate() * v_code;  // + data bit (r = +1 * nco_code_replica)
    } else {
      S = X.conjugate() * -v_code;  // - data bit (r = -1 * nco_code_replica)
    }

    // propagate weights
    W += mu * (S - Phi * W);
    // W *= std::conj(W(0)) / std::abs(W(0));  // enforce phase alignment

    // early
    // v_code = code[static_cast<int>(std::fmod(rem_code_phase + t_space, 1023.0))] ? 1.0 : -1.0;
    v_code = code[static_cast<int>(std::round(rem_code_phase + t_space)) % 1023] ? 1.0 : -1.0;
    E += (v_code * y);

    // late
    // v_code = code[static_cast<int>(std::fmod(rem_code_phase - t_space, 1023.0))] ? 1.0 : -1.0;
    v_code = code[static_cast<int>(std::round(rem_code_phase - t_space)) % 1023] ? 1.0 : -1.0;
    L += (v_code * y);

    // increment
    rem_code_phase += d_code;
    rem_carr_phase += d_carr;
    samp_remaining--;
  }
}

void LmsNormalize(Eigen::Ref<Eigen::VectorXcd> W) {
  W *= std::conj(W(0)) / std::abs(W(0));
  // W /= std::abs(W(0));
}

}  // namespace sturdr