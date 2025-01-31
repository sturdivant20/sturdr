/**
 * *vector-tracking.hpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/vector-tracking.hpp
 * @brief   Handles key vector tracking functionality
 * @date    January 2025
 * @ref     1. "GPS Carrier Phase Tracking in Difficult Environments Using Vector Tracking For
 *              Precise Positioning and Vehicle Attitude Estimation" (Ph.D. Dissertation, Auburn
 *              University) - Scott Martin
 *          2. "Modeling and Performance Analysis of GPS Vector Tracking Algorithms" (Ph.D
 *              Dissertation, Auburn University) - Matthew Lashley
 *          3. "A GPS and GLONASS L1 Vector Tracking Software-Defined Receiver" (Masters Thesis,
 *              Auburn University) - Tanner Watts
 *          4. "A GPS L5 Software Defined Vector Tracking Receiver" (Masters Thesis, Auburn
 *              University) - C. Anderson Givhan
 * =======  ========================================================================================
 */

#ifndef STURDR_VECTOR_TRACKING_HPP
#define STURDR_VECTOR_TRACKING_HPP

#include <cmath>
#include <satutils/ephemeris.hpp>
#include <sturdins/kns.hpp>

#include "sturdr/structs-enums.hpp"

namespace sturdr {

/**
 * *=== VectorDllNco *===
 * @brief NCO frequency update for the DLL of a channel
 * @param chip_rate Nominal chipping rate of the signal [chips/s]
 * @param T         Nominal integration period [s]
 * @param theta     Remainder code phase from previous NCO correction [chips]
 * @param tR        Current receive time [s]
 * @param tR_pred   Predicted next receive time [s]
 */
double VectorDllNco(double &chip_rate, double &T, double &theta, double &tR, double &tR_pred);

/**
 * *=== VectorFllNco *===
 * @brief NCO frequency update for the FLL of a channel
 * @param intmd_freq  intermediate frequency of the receiver front end [rad/s]
 * @param lambda      carrier wavelength [m/rad]
 * @param psrdot      predicted pseudorange-rate measurement [m/s]
 */
double VectorFllNco(double &intmd_freq, double &lambda, double &psrdot);

/**
 * *=== RunVDFllUpdate ===*
 * @brief Runs the vector Delay-Frequency lock loop navigation update
 * @param Eph       Satellite ephemeris object
 * @param tR        Receive time [s]
 * @param tT        Channel transmit time [s]
 * @param doppler   Channel doppler [rad/s]
 * @param dll_disc  Channel DLL discriminator [chips]
 * @param fll_disc  Channel FLL discriminator [rad/s]
 * @param n_samp    Number of samples elapsed since most recent vector update
 * @param filt      EKF filter object
 * @param T         Nominal integration period [s]
 * @param chip_rate Nominal chipping rate of the signal [chips/s]
 * @param theta     Remainder code phase from previous NCO correction [chips]
 * @param lambda    carrier wavelength [m/rad]
 * @param beta      chip width [m/chip]
 */
// VectorNcoUpdate RunVDFllUpdate(
//     satutils::KeplerEphem<double> &Eph,
//     double &tR,
//     double &tT,
//     double &doppler,
//     double &dll_disc,
//     double &fll_disc,
//     uint64_t &n_samp,
//     sturdins::Kns &filt,
//     double &T,
//     double &samp_freq,
//     double &chip_rate,
//     double &theta,
//     double &lambda,
//     double &beta);
void RunVDFllUpdate(
    uint64_t &d_samp,
    double &samp_freq,
    double &intmd_freq,
    ChannelNavData &data,
    double &tR,
    double &T,
    sturdins::Kns &filt);

}  // namespace sturdr

#endif