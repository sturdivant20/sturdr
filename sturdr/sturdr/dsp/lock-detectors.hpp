/**
 * *lock-detectors.hpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/dsp/lock-detectors.hpp
 * @brief   Satellite acquisition methods.
 * @date    December 2024
 * @author  Daniel Sturdivant <sturdivant20@gmail.com>
 * @ref     1. "Understanding GPS/GNSS Principles and Applications", 3rd Edition, 2017
 *              - Kaplan & Hegarty
 *          2. "Global Positioning System: Theory and Applications, Volume 1", 1996
 *              - Spilker, Axlerad, Parkinson, Enge
 *          2. "Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems", 2nd
 *              Edition, 2013 - Groves
 *          3. "Global Positioning System: Signals, Measurements, and Performance", 2nd Edition,
 *              2006 - Misra & Enge
 * =======  ========================================================================================
 */

#ifndef STURDR_LOCK_DETECTORS_HPP
#define STURDR_LOCK_DETECTORS_HPP

#include <complex>

namespace sturdr {

/**
 * *=== CodeLockDetector ===*
 * @brief Code lock detector and Carrier-to-Noise ratio estimator (must be reset if integration
 *        period changes
 * @param code_lock
 * @param cno
 * @param PC
 * @param PN
 * @param P_old
 * @param P_new
 * @param T
 * @param alpha
 */
void CodeLockDetector(
    bool &code_lock,
    double &cno,
    double &PC,
    double &PN,
    const std::complex<double> &P_old,
    const std::complex<double> &P_new,
    const double &T,
    const double &alpha = 0.005);
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
    const double &alpha = 0.005);

/**
 * *=== CarrierLockDetector ===*
 * @brief Carrier phase lock detector (Spilker, Axlerad, Parkinson, Enge)
 * @param carr_lock Current carrier lock status
 * @param NBD       Narrow band difference memory
 * @param NBP       Narrow band power memory
 * @param P         Current prompt correlator values
 * @param alpha     Smoothing (filtering) coefficient, by default 5e-3
 */
void CarrierLockDetector(
    bool &carr_lock,
    double &NBD,
    double &NBP,
    const std::complex<double> &P,
    const double &alpha = 0.005);
void CarrierLockDetector(
    bool &carr_lock,
    double &NBD,
    double &NBP,
    const double &IP,
    const double &QP,
    const double &alpha = 0.005);

/**
 * *=== LockDetectors ===*
 * @brief Code and carrier GNSS lock detectors
 * @param code_lock Current code lock status
 * @param carr_lock Current carrier lock status
 * @param cno       Current carrier-to-noise-density ratio magnitude (not dB-Hz)
 * @param NBD       Narrow band difference memory
 * @param NBP       Narrow band power memory
 * @param PC        Carrier power memory
 * @param PN        Noise power memory
 * @param P_old     Previous prompt correlator values
 * @param P_new     Current prompt correlator values
 * @param T         Integration time [s]
 * @param alpha     Smoothing (filtering) coefficient, by default 5e-3
 */
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
    const double &alpha = 0.005);
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
    const double &alpha = 0.005);

/**
 * *=== LowPassFilter ===*
 * @brief Simple implementation of a moving average (low-pass) filter
 * @param prev  Old value
 * @param curr  New value
 * @param alpha Forgetting coefficient, by default 0.005
 * @return Filtered value
 */
double LowPassFilter(const double &prev, const double &curr, const double &alpha = 0.005);

}  // end namespace sturdr

#endif