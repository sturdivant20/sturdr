/**
 * *discriminator.cpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/dsp/discriminator.cpp
 * @brief   Standard satellite tracking match filter discriminators.
 * @date    December 2024
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

#include "sturdr/dsp/lock-detectors.hpp"

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
    spdlog::default_logger()->error(
        "lock-detectors.cpp CodeLockDetector failed! Error -> {}", e.what());
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
    spdlog::default_logger()->error(
        "lock-detectors.cpp CodeLockDetector failed! Error -> {}", e.what());
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
    spdlog::default_logger()->error(
        "lock-detectors.cpp CarrierLockDetector failed! Error -> {}", e.what());
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
    spdlog::default_logger()->error(
        "lock-detectors.cpp CarrierLockDetector failed! Error -> {}", e.what());
  }
}

// *=== LockDetectors ===*
void LockDetectors(
    bool &code_lock,
    bool &carr_lock,
    double &cno,
    double &NBD,
    double &NBP,
    double &PC,
    double &PN,
    const std::complex<double> &P_old,
    const std::complex<double> &P_new,
    const double &T,
    const double &alpha) {
  try {
    // Calculate powers
    double I = P_new.real() * P_new.real();
    double Q = P_new.imag() * P_new.imag();
    double P_prev = std::norm(P_old);
    double P_curr = I + Q;
    double P_diff = I - Q;
    double P_avg = 0.5 * (P_prev + P_curr);
    double P_noise = std::pow(std::sqrt(P_curr) - std::sqrt(P_prev), 2);

    // Carrier lock detection
    //  -> cos(2 * φ) = NBD / NBP
    //  -> Phase error should be less than 15° to be considered locked
    //  -> cos(2 * 15) ~ 0.866
    NBD = LowPassFilter(NBD, P_diff, alpha);
    NBP = LowPassFilter(NBP, P_curr);
    carr_lock = ((NBD / NBP) > 0.866);

    // Code lock detection (based on Beaulieu's Method)
    // -> For good code lock, C/N0 should be above 30 dB-Hz or a magnitude of 1000
    PC = LowPassFilter(PC, P_avg, alpha);
    PN = LowPassFilter(PN, P_noise, alpha);
    cno = PC / (PN * T);  // (1 / (PN / PC)) / T
    code_lock = (cno > 1000.0);

  } catch (std::exception &e) {
    spdlog::default_logger()->error(
        "lock-detectors.cpp LockDetectors failed! Error -> {}", e.what());
  }
}
void LockDetectors(
    bool &code_lock,
    bool &carr_lock,
    double &cno,
    double &NBD,
    double &NBP,
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
    double I = IP_new * IP_new;
    double Q = QP_new * QP_new;
    double P_prev = IP_old * IP_old + QP_old * QP_old;
    double P_curr = I + Q;
    double P_diff = I - Q;
    double P_avg = 0.5 * (P_prev + P_curr);
    double P_noise = std::pow(std::sqrt(P_curr) - std::sqrt(P_prev), 2);

    // Carrier lock detection
    //  -> cos(2 * φ) = NBD / NBP
    //  -> Phase error should be less than 15° to be considered locked
    //  -> cos(2 * 15) ~ 0.866
    NBD = LowPassFilter(NBD, P_diff, alpha);
    NBP = LowPassFilter(NBP, P_curr);
    carr_lock = ((NBD / NBP) > 0.866);

    // Code lock detection (based on Beaulieu's Method)
    // -> For good code lock, C/N0 should be above 30 dB-Hz or a magnitude of 1000
    PC = LowPassFilter(PC, P_avg, alpha);
    PN = LowPassFilter(PN, P_noise, alpha);
    cno = PC / (PN * T);  // (1 / (PN / PC)) / T
    code_lock = (cno > 1000.0);

  } catch (std::exception &e) {
    spdlog::default_logger()->error(
        "lock-detectors.cpp LockDetectors failed! Error -> {}", e.what());
  }
}

// *=== LowPassFilter ===
double LowPassFilter(const double &prev, const double &curr, const double &alpha) {
  try {
    // out = (1.0 - alpha) * prev + alpha * curr
    return prev + alpha * (curr - prev);

  } catch (std::exception &e) {
    spdlog::default_logger()->error(
        "lock-detectors.cpp LowPassFilter failed! Error -> {}", e.what());
    return std::nan("1");
  }
}

}  // end namespace sturdr