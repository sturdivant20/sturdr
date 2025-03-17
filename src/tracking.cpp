/**
 * *tracking.cpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/tracking.cpp
 * @brief   Satellite scalar tracking methods.
 * @date    January 2025
 * @author  Daniel Sturdivant <sturdivant20@gmail.com>
 * @ref     1. "Understanding GPS/GNSS Principles and Applications", 3rd Edition, 2017
 *              - Kaplan & Hegarty
 *          2. "Are PLLs Dead? A Tutorial on Kalman Filter-Based Techniques for Digital Carrier
 *              Syncronization" - Vila-Valls, Closas, Navarro, Fernandez-Prades
 *          3. "A Software-Defined GPS and Galileo Receiver: A Single-Frequency Approach", 2007
 *              - Borre, Akos, Bertelsen, Rinder, Jensen
 *          4. "Position, Navigation, and Timing Technologies in the 21st Century", Volume 1, 2021
 *              - Morton, Diggelen, Spilker Jr., Parkinson
 * =======  ========================================================================================
 */

#include "sturdr/tracking.hpp"

#include <spdlog/spdlog.h>

#include <cmath>
#include <exception>
#include <navtools/constants.hpp>

#include "sturdr/discriminator.hpp"

namespace sturdr {

inline static const double a = 1.4142;
inline static const double b = 1.1;
inline static const double c = 2.4;
inline static const Eigen::Matrix<double, 5, 5> I5 = Eigen::Matrix<double, 5, 5>::Identity();

// *=== NaturalFrequency ===*
double NaturalFrequency(const double &bw, const int order) {
  try {
    if (order == 1) {
      return bw / 4.0;
    } else if (order == 2) {
      return bw * 8.0 / 3.0;
    } else if (order == 3) {
      return bw * 6.56 / 5.146;
    } else {
      throw std::invalid_argument("Input order must be 1,2, or 3!");
    }

  } catch (std::exception &e) {
    spdlog::get("sturdr-console")
        ->error("tracking.cpp NaturalFrequency failed! Error -> {}", e.what());
    return std::nan("1");
  }
}

// *=== FLLAssistedPLL_2ndOrder ===*
void FLLassistedPLL_2ndOrder(
    double &nco_freq,
    double &vel_accum,
    const double &phase_err,
    const double &freq_err,
    const double &T,
    const double &w0p,
    const double &w0f) {
  try {
    double vel_accum_k = vel_accum + T * (phase_err * w0p * w0p + freq_err * w0f);
    nco_freq = 0.5 * (vel_accum_k + vel_accum) + (phase_err * a * w0p);
    vel_accum = vel_accum_k;

  } catch (std::exception &e) {
    spdlog::get("sturdr-console")
        ->error("tracking.cpp FLLassistedPLL_2ndOrder failed! Error -> {}", e.what());
  }
}

// *=== FLLAssistedFll_3rdOrder ===*
void FLLassistedPLL_3rdOrder(
    double &nco_freq,
    double &vel_accum,
    double &acc_accum,
    const double &phase_err,
    const double &freq_err,
    const double &T,
    const double &w0p,
    const double &w0f) {
  try {
    double w0p2 = w0p * w0p;
    double acc_accum_k = acc_accum + T * (w0p2 * w0p * phase_err + w0f * w0f * freq_err);
    double vel_accum_k = vel_accum + T * (0.5 * (acc_accum_k + acc_accum) + b * w0p2 * phase_err +
                                          a * w0f * freq_err);
    nco_freq = 0.5 * (vel_accum_k + vel_accum) + c * w0p * phase_err;
    vel_accum = vel_accum_k;
    acc_accum = acc_accum_k;

  } catch (std::exception &e) {
    spdlog::get("sturdr-console")
        ->error("tracking.cpp FLLassistedPLL_3rdOrder failed! Error -> {}", e.what());
  }
}

// *=== PLL_2ndOrder ===*
void PLL_2ndOrder(
    double &nco_freq,
    double &vel_accum,
    const double &phase_err,
    const double &T,
    const double &w0) {
  try {
    double vel_accum_k = vel_accum + T * w0 * w0 * phase_err;
    nco_freq = 0.5 * (vel_accum_k + vel_accum) + a * w0 * phase_err;
    vel_accum = vel_accum_k;

  } catch (std::exception &e) {
    spdlog::get("sturdr-console")->error("tracking.cpp PLL_2ndOrder failed! Error -> {}", e.what());
  }
}

// *=== PLL_3rdOrder ===*
void PLL_3rdOrder(
    double &nco_freq,
    double &vel_accum,
    double &acc_accum,
    const double &phase_err,
    const double &T,
    const double &w0) {
  try {
    double w02 = w0 * w0;
    double acc_accum_k = acc_accum + T * w02 * w0 * phase_err;
    double vel_accum_k = vel_accum + T * (0.5 * (acc_accum_k + acc_accum) + b * w02 * phase_err);
    nco_freq = 0.5 * (vel_accum_k + vel_accum) + c * w0 * phase_err;
    vel_accum = vel_accum_k;
    acc_accum = acc_accum_k;

  } catch (std::exception &e) {
    spdlog::get("sturdr-console")->error("tracking.cpp PLL_3rdOrder failed! Error -> {}", e.what());
  }
}

//! ------------------------------------------------------------------------------------------------

// *=== TrackingKF ===*
TrackingKF::TrackingKF()
    : P_{Eigen::Matrix<double, 5, 5>::Zero()},
      Q_{Eigen::Matrix<double, 5, 5>::Zero()},
      R_{Eigen::Matrix<double, 3, 3>::Zero()},
      F_{Eigen::Matrix<double, 5, 5>::Identity()},
      G_{Eigen::Matrix<double, 5, 2>::Zero()},
      H_{Eigen::Matrix<double, 3, 5>::Zero()} {};

// *=== ~TrackingKF ===*
TrackingKF::~TrackingKF(){};

void TrackingKF::Init(
    const double &init_carr_phase,
    const double &init_carr_doppler,
    const double &init_code_phase,
    const double &intmd_freq,
    const double &code_freq) {
  // Initialize state and covariance
  x_ << init_carr_phase, init_carr_doppler, 0.0, init_code_phase, 0.0;
  // P_.diagonal() << navtools::PI_SQU<double> / 3.0, navtools::PI_SQU<> * 125e3, 1000.0, 1.0,
  // 100.0;
  P_.diagonal() << navtools::PI_SQU<double> / 3.0, navtools::PI_SQU<> * 62.5e3,
      navtools::PI_SQU<> * 1e3, 5e-1, 5e2;

  // Initialize dynamic model
  // make_F(init_k, init_T);
  // make_G(init_T);
  // make_Q(init_w0d, init_w0p, init_w0f, init_k, init_T);
  u_ << intmd_freq, code_freq;

  // Initialize measurement model
  make_H();
  // make_R(init_cno, init_T);
}

// *=== UpdateDynamicsParam ===*
void TrackingKF::UpdateDynamicsParam(
    const double &w0d, const double &w0p, const double &w0f, const double &k, const double &T) {
  make_F(k, T);
  make_G(T);
  make_Q(w0d, w0p, w0f, k, T);
}

// *=== UpdateMeasurementsParam ===*
void TrackingKF::UpdateMeasurementsParam(
    const double &dll_var, const double &pll_var, const double &fll_var) {
  // make_H();
  // make_R(cno, T);
  R_.diagonal() << pll_var, fll_var, dll_var;
}

// *=== Run ===*
void TrackingKF::Run(double &chip_err, double &phase_err, double &freq_err) {
  try {
    // Prediction
    x_ = F_ * x_ + G_ * u_;
    P_ = F_ * P_ * F_.transpose() + Q_;

    // Correction
    Eigen::Vector<double, 3> dz{phase_err, freq_err, chip_err};
    Eigen::Matrix<double, 5, 3> PHt = P_ * H_.transpose();
    Eigen::Matrix<double, 5, 3> K = PHt * (H_ * PHt + R_).inverse();
    P_ = (I5 - K * H_) * P_;
    x_ += K * dz;
  } catch (std::exception &e) {
    spdlog::get("sturdr-console")
        ->error("tracking.cpp TrackingKF::Run failed! Error -> {}", e.what());
    // return std::nan("1") * Eigen::Vector<double, 5>::Ones();
  }

  // return x_;
}

void TrackingKF::SetRemCarrierPhase(const double &rem_carrier_phase) {
  x_(0) = rem_carrier_phase;
}
void TrackingKF::SetRemCodePhase(const double &rem_code_phase) {
  x_(3) = rem_code_phase;
}
double TrackingKF::GetRemCarrierPhase() {
  return x_(0);
}
double TrackingKF::GetRemCodePhase() {
  return x_(3);
}
double TrackingKF::GetDoppler() {
  return x_(1);
}

// *=== make_F ===*
void TrackingKF::make_F(const double &k, const double &T) {
  // double T2 = T * T;
  // clang-format off
  // F_ << 1.0,     T,     0.5 * T2, 0.0, 0.0,
  //       0.0,   1.0,            T, 0.0, 0.0,
  //       0.0,   0.0,          1.0, 0.0, 0.0,
  //       0.0, k * T, 0.5 * k * T2, 1.0,   T,
  //       0.0,   0.0,          0.0, 0.0, 1.0;
  // clang-format on
  F_(0, 1) = T;
  F_(0, 2) = 0.5 * T * T;
  F_(1, 2) = T;
  F_(3, 1) = k * T;
  F_(3, 2) = 0.5 * F_(3, 1) * T;
  F_(3, 4) = T;
}

// *=== make_F ===*
void TrackingKF::make_G(const double &T) {
  // clang-format off
  // G_ <<   T, 0.0,
  //       0.0, 0.0,
  //       0.0, 0.0,
  //       0.0,   T,
  //       0.0, 0.0;
  // clang-format on
  G_(0, 0) = T;
  G_(3, 1) = T;
}

// *=== make_Q ===*
void TrackingKF::make_Q(
    const double &w0d, const double &w0p, const double &w0f, const double &k, const double &T) {
  double T2 = T * T;
  double T3 = T * T2;
  // double T4 = T * T3;
  // double T5 = T * T4;
  // double k2 = k * k;
  // clang-format off
  // Q_ <<       (w0f*T5)/20. + (w0p*T3)/3.,   (T2*(w0f*T2 + 4.*w0p))/8.,   (T3*w0f)/6.,               (T3*k*(3.*w0f*T2 + 2.0*w0p))/60.,          0.,
  //              (w0f*T4)/8. + (w0p*T2)/2.,         (w0f*T3)/3. + w0p*T,   (T2*w0f)/2.,                    (T2*k*(w0f*T2 + 4.*w0p))/8.,          0.,
  //                            (T3*w0f)/6.,                 (T2*w0f)/2.,         T*w0f,                                  (T3*k*w0f)/6.,          0.,
  //       (T3*k*(3.*w0f*T2 + 20.*w0p))/60., (T2*k*(w0f*T2 + 4.*w0p))/8., (T3*k*w0f)/6., (T3*(3.*w0f*T2*k2 + 20.*w0p*k2 + 20.*w0d))/60., (T2*w0d)/2.,
  //                                     0.,                          0.,            0.,                                    (T2*w0d)/2.,       T*w0d;
  // clang-format on
  Q_(0, 0) = T3 * ((w0f * T2) / 20.0 + w0p / 3.0);
  Q_(0, 1) = T2 * ((w0f * T2) / 8.0 + w0p / 2.0);
  Q_(0, 2) = (T3 * w0f) / 6.0;
  Q_(0, 3) = (T3 * k * (3.0 * w0f * T2 + 2.0 * w0p)) / 60.0;
  Q_(1, 0) = Q_(0, 1);
  Q_(1, 1) = (w0f * T3) / 3. + w0p * T;
  Q_(1, 2) = (T2 * w0f) / 2.0;
  Q_(1, 3) = (T2 * k * (w0f * T2 + 4. * w0p)) / 8.0;
  Q_(2, 0) = Q_(0, 1);
  Q_(2, 1) = Q_(1, 2);
  Q_(2, 2) = T * w0f;
  Q_(2, 3) = (T3 * k * w0f) / 6.0;
  Q_(3, 0) = Q_(0, 3);
  Q_(3, 1) = Q_(1, 3);
  Q_(3, 2) = Q_(2, 3);
  Q_(3, 3) = (T3 * k * k * (3.0 * w0f * T2 + 20.0 * (w0p + w0d))) / 60.0;
  Q_(3, 4) = (T2 * w0d) / 2.0;
  Q_(4, 3) = Q_(3, 4);
  Q_(4, 4) = T * w0d;
}

// *=== make_H ===*
void TrackingKF::make_H() {
  // // clang-format off
  // H_ << 1.0, 0.0, 0.0, 0.0, 0.0,
  //       0.0, 1.0, 0.0, 0.0, 0.0,
  //       0.0, 0.0, 0.0, 1.0, 0.0;
  // // clang-format on
  H_(0, 0) = 1.0;
  H_(1, 1) = 1.0;
  H_(2, 3) = 1.0;
}

// *=== make_R ===*
void TrackingKF::make_R(const double &cno, const double &T) {
  R_.diagonal() << PllVariance(cno, T), FllVariance(cno, T), DllVariance(cno, T);
}

}  // end namespace sturdr