/**
 * *lock-detectors.cpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/lock-detectors.cpp
 * @brief   Satellite code and carrier lock detectors.
 * @date    January 2025
 * @author  Daniel Sturdivant <sturdivant20@gmail.com>
 * @ref     1. "Understanding GPS/GNSS Principles and Applications", 3rd Edition, 2017
 *              - Kaplan & Hegarty
 *          2. "Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems", 2nd
 *              Edition, 2013 - Groves
 *          3. "Global Positioning System: Signals, Measurements, and Performance", 2nd Edition,
 *              2006 - Misra & Enge
 *          4. "Position, Navigation, and Timing Technologies in the 21st Century", Volume 1, 2021
 *              - Morton, Diggelen, Spilker Jr., Parkinson
 * =======  ========================================================================================
 */

#include "sturdr/lock-detectors.hpp"

#include <spdlog/spdlog.h>

#include <cmath>
#include <exception>

namespace sturdr {

// *=== CodeLockDetector ===*
void CodeLockDetector(
    bool &code_lock,
    double &cno,
    double &PC,
    double &PN,
    const std::complex<double> &P_old,
    const std::complex<double> &P_new,
    const double &T,
    const double &alpha) {
  try {
    // Calculate powers
    double P_prev = std::norm(P_old);
    double P_curr = std::norm(P_new);
    double P_avg = 0.5 * (P_prev + P_curr);
    double P_noise = std::pow(std::sqrt(P_curr) - std::sqrt(P_prev), 2);

    // Code lock detection (based on Beaulieu's Method)
    // -> For good code lock, C/N0 should be above 30 dB-Hz or a magnitude of 1000
    PC = LowPassFilter(PC, P_avg, alpha);
    PN = LowPassFilter(PN, P_noise, alpha);
    cno = PC / (PN * T);  // (1 / (PN / PC)) / T
    code_lock = (cno > 1000.0);

  } catch (std::exception &e) {
    spdlog::get("sturdr-console")
        ->error("lock-detectors.cpp CodeLockDetector failed! Error -> {}", e.what());
  }
}
void CodeLockDetector(
    bool &code_lock,
    double &cno,
    double &PC,
    double &PN,
    const double &IP_old,
    const double &QP_old,
    const double &IP_new,
    const double &QP_new,
    const double &T,
    const double &alpha) {
  try {
    // Calculate powers
    double P_prev = IP_old * IP_old + QP_old * QP_old;
    double P_curr = IP_new * IP_new + QP_new * QP_new;
    double P_avg = 0.5 * (P_prev + P_curr);
    double P_noise = std::pow(std::sqrt(P_curr) - std::sqrt(P_prev), 2);

    // Code lock detection (based on Beaulieu's Method)
    // -> For good code lock, C/N0 should be above 30 dB-Hz or a magnitude of 1000
    PC = LowPassFilter(PC, P_avg, alpha);
    PN = LowPassFilter(PN, P_noise, alpha);
    cno = PC / (PN * T);  // (1 / (PN / PC)) / T
    code_lock = (cno > 1000.0);

  } catch (std::exception &e) {
    spdlog::get("sturdr-console")
        ->error("lock-detectors.cpp CodeLockDetector failed! Error -> {}", e.what());
  }
}

// *=== CarrierLockDetector ===*
void CarrierLockDetector(
    bool &carr_lock, double &NBD, double &NBP, const std::complex<double> &P, const double &alpha) {
  try {
    // Calculate powers
    double I = P.real() * P.real();
    double Q = P.imag() * P.imag();
    double P_curr = I + Q;
    double P_diff = I - Q;

    // Carrier lock detection
    //  -> cos(2 * φ) = NBD / NBP
    //  -> Phase error should be less than 15° to be considered locked
    //  -> cos(2 * 15) ~ 0.866
    NBD = LowPassFilter(NBD, P_diff, alpha);
    NBP = LowPassFilter(NBP, P_curr);
    carr_lock = ((NBD / NBP) > 0.866);

  } catch (std::exception &e) {
    spdlog::get("sturdr-console")
        ->error("lock-detectors.cpp CarrierLockDetector failed! Error -> {}", e.what());
  }
}
void CarrierLockDetector(
    bool &carr_lock,
    double &NBD,
    double &NBP,
    const double &IP,
    const double &QP,
    const double &alpha) {
  try {
    // Calculate powers
    double I = IP * IP;
    double Q = QP * QP;
    double P_curr = I + Q;
    double P_diff = I - Q;

    // Carrier lock detection
    //  -> cos(2 * φ) = NBD / NBP
    //  -> Phase error should be less than 15° to be considered locked
    //  -> cos(2 * 15) ~ 0.866
    NBD = LowPassFilter(NBD, P_diff, alpha);
    NBP = LowPassFilter(NBP, P_curr);
    carr_lock = ((NBD / NBP) > 0.866);

  } catch (std::exception &e) {
    spdlog::get("sturdr-console")
        ->error("lock-detectors.cpp CarrierLockDetector failed! Error -> {}", e.what());
  }
}

// // *=== LockDetectors ===*
// void LockDetectors(
//     bool &code_lock,
//     bool &carr_lock,
//     double &cno,
//     double &NBD,
//     double &NBP,
//     double &PC,
//     double &PN,
//     const std::complex<double> &P_old,
//     const std::complex<double> &P_new,
//     const double &T,
//     const double &alpha) {
//   try {
//     // Calculate powers
//     double I = P_new.real() * P_new.real();
//     double Q = P_new.imag() * P_new.imag();
//     double P_prev = std::norm(P_old);
//     double P_curr = I + Q;
//     double P_diff = I - Q;
//     double P_avg = 0.5 * (P_prev + P_curr);
//     double P_noise = std::pow(std::sqrt(P_curr) - std::sqrt(P_prev), 2);

//     // Carrier lock detection
//     //  -> cos(2 * φ) = NBD / NBP
//     //  -> Phase error should be less than 15° to be considered locked
//     //  -> cos(2 * 15) ~ 0.866
//     NBD = LowPassFilter(NBD, P_diff, alpha);
//     NBP = LowPassFilter(NBP, P_curr);
//     carr_lock = ((NBD / NBP) > 0.866);

//     // Code lock detection (based on Beaulieu's Method)
//     // -> For good code lock, C/N0 should be above 30 dB-Hz or a magnitude of 1000
//     PC = LowPassFilter(PC, P_avg, alpha);
//     PN = LowPassFilter(PN, P_noise, alpha);
//     cno = PC / (PN * T);  // (1 / (PN / PC)) / T
//     code_lock = (cno > 1000.0);

//   } catch (std::exception &e) {
//     spdlog::get("sturdr-console")
//         ->error("lock-detectors.cpp LockDetectors failed! Error -> {}", e.what());
//   }
// }
// void LockDetectors(
//     bool &code_lock,
//     bool &carr_lock,
//     double &cno,
//     double &NBD,
//     double &NBP,
//     double &PC,
//     double &PN,
//     const double &IP_old,
//     const double &QP_old,
//     const double &IP_new,
//     const double &QP_new,
//     const double &T,
//     const double &alpha) {
//   try {
//     // Calculate powers
//     double I = IP_new * IP_new;
//     double Q = QP_new * QP_new;
//     double P_prev = IP_old * IP_old + QP_old * QP_old;
//     double P_curr = I + Q;
//     double P_diff = I - Q;
//     double P_avg = 0.5 * (P_prev + P_curr);
//     double P_noise = std::pow(std::sqrt(P_curr) - std::sqrt(P_prev), 2);

//     // Carrier lock detection
//     //  -> cos(2 * φ) = NBD / NBP
//     //  -> Phase error should be less than 15° to be considered locked
//     //  -> cos(2 * 15) ~ 0.866
//     NBD = LowPassFilter(NBD, P_diff, alpha);
//     NBP = LowPassFilter(NBP, P_curr);
//     carr_lock = ((NBD / NBP) > 0.866);

//     // Code lock detection (based on Beaulieu's Method)
//     // -> For good code lock, C/N0 should be above 30 dB-Hz or a magnitude of 1000
//     PC = LowPassFilter(PC, P_avg, alpha);
//     PN = LowPassFilter(PN, P_noise, alpha);
//     cno = PC / (PN * T);  // (1 / (PN / PC)) / T
//     code_lock = (cno > 1000.0);

//   } catch (std::exception &e) {
//     spdlog::get("sturdr-console")
//         ->error("lock-detectors.cpp LockDetectors failed! Error -> {}", e.what());
//   }
// }

LockDetectors::LockDetectors(double alpha)
    : k_{0.0},
      nbp_{0.0},
      nbd_{0.0},
      m2_{0.0},
      m4_{0.0},
      A_{0.0},
      eta_{0.0},
      cno_{1000.0},
      carr_ratio_{0.0},
      alpha_{alpha},
      dt_{0.0},
      P_prev_{0.0} {};
LockDetectors::~LockDetectors() = default;

void LockDetectors::Update(const double &IP, const double &QP, const double &T) {
  double I = IP * IP;
  double Q = QP * QP;
  double P_curr = I + Q;
  double P_diff = I - Q;

  if (dt_ != T) {
    Reset();
    dt_ = T;
  }

  // double P_curr2 = P_curr * P_curr;
  // if (k_ < 200.0) {
  //   k_ += 1.0;
  //   double g = 1.0 / k_;
  //   m2_ = LowPassFilter(m2_, P_curr, g);
  //   m4_ = LowPassFilter(m4_, P_curr2, g);
  //   nbd_ = LowPassFilter(nbd_, P_diff, g);
  //   nbp_ = LowPassFilter(nbp_, P_curr, g);
  // } else {
  //   m2_ = LowPassFilter(m2_, P_curr, alpha_);
  //   m4_ = LowPassFilter(m4_, P_curr2, alpha_);
  //   double pd = std::sqrt(std::abs(2.0 * m2_ * m2_ - m4_));
  //   double pn = std::abs(m2_ - pd);
  //   cno_ = (pd / pn) / T;
  //   carr_ratio_ = nbd_ / nbp_;
  // }

  double P_avg = 0.5 * (P_prev_ + P_curr);
  double P_noise = std::pow(std::sqrt(P_curr) - std::sqrt(P_prev_), 2);
  if (k_ < 100.0) {
    k_ += 1.0;
    double g = 1.0 / k_;
    m2_ = LowPassFilter(m2_, P_avg, g);
    m4_ = LowPassFilter(m4_, P_noise, g);
    nbd_ = LowPassFilter(nbd_, P_diff, g);
    nbp_ = LowPassFilter(nbp_, P_curr, g);
  } else {
    nbd_ = LowPassFilter(nbd_, P_diff, alpha_);
    nbp_ = LowPassFilter(nbp_, P_curr, alpha_);
    m2_ = LowPassFilter(m2_, P_avg, alpha_);
    m4_ = LowPassFilter(m4_, P_noise, alpha_);
    cno_ = m2_ / (m4_ * T);  // (1 / (PN / PC)) / T
    carr_ratio_ = nbd_ / nbp_;
  }

  P_prev_ = P_curr;
}
void LockDetectors::Update(const std::complex<double> &P, const double &T) {
  Update(P.real(), P.imag(), T);
}

void LockDetectors::Update(
    const double &IP,
    const double &QP,
    const Eigen::Ref<const Eigen::Vector<double, 12>> &IN,
    const Eigen::Ref<const Eigen::Vector<double, 12>> &QN,
    const double &T) {
  if (dt_ != T) {
    Reset();
    dt_ = T;
  }
  double I = IP * IP;
  double Q = QP * QP;
  double nbp = I + Q;
  double nbd = I - Q;
  // double sigma_sq = (IN.array() * IN.array() + QN.array() * QN.array()).mean();
  double sigma_sq = 0.5 * ((IN.array() * IN.array()).mean() + (QN.array() * QN.array()).mean());
  if (k_ < 100.0) {
    k_ += 1.0;
    double g = 1.0 / k_;
    A_ = LowPassFilter(A_, nbp, g);
    eta_ = LowPassFilter(eta_, sigma_sq, g);
    nbd_ = LowPassFilter(nbd_, nbd, g);
    nbp_ = LowPassFilter(nbp_, nbd, g);
  } else {
    A_ = LowPassFilter(A_, nbp, alpha_);
    eta_ = LowPassFilter(eta_, sigma_sq, alpha_);
    nbd_ = LowPassFilter(nbd_, nbd, alpha_);
    nbp_ = LowPassFilter(nbp_, nbd, alpha_);
    cno_ = (A_ - 2.0 * eta_) / (2.0 * T * eta_);
    carr_ratio_ = nbd_ / nbp_;
  }

  P_prev_ = nbp;
}
void LockDetectors::Update(
    const std::complex<double> &P,
    const Eigen::Ref<const Eigen::Vector<std::complex<double>, 12>> &N,
    const double &T) {
  Update(P.real(), P.imag(), N.real(), N.imag(), T);
}

void LockDetectors::Reset() {
  k_ = 0.0;
  m2_ = 0.0;
  m4_ = 0.0;
  nbd_ = 0.0;
  nbp_ = 0.0;
  A_ = 0.0;
  eta_ = 0.0;
}

bool LockDetectors::GetCodeLock() {
  // Code lock detection
  // -> For good code lock, C/N0 should be above 30 dB-Hz or a magnitude of 1000
  return (cno_ > 1000.0);
}

bool LockDetectors::GetCarrierLock() {
  // Carrier lock detection
  //  -> cos(2 * φ) = NBD / NBP
  //  -> Phase error should be less than 15° to be considered locked
  //  -> cos(2 * 15) ~ 0.866
  return (carr_ratio_ > 0.866);
}

double LockDetectors::GetCno() {
  return cno_;
}

// *=== LowPassFilter ===
double LowPassFilter(const double &prev, const double &curr, const double &alpha) {
  try {
    // out = (1.0 - alpha) * prev + alpha * curr
    return prev + alpha * (curr - prev);

  } catch (std::exception &e) {
    spdlog::get("sturdr-console")
        ->error("lock-detectors.cpp LowPassFilter failed! Error -> {}", e.what());
    return std::nan("1");
  }
}

}  // end namespace sturdr