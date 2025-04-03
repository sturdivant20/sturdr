/**
 * *gnss-signal.hpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/gnss-signal.hpp
 * @brief   Common GNSS signal functions.
 * @date    January 2025
 * @author  Daniel Sturdivant <sturdivant20@gmail.com>
 * @ref     1. "A Software-Defined GPS and Galileo Receiver: A Single-Frequency Approach", 2007
 *              - Borre, Akos, Bertelsen, Rinder, Jensen
 *          2. "Understanding GPS/GNSS Principles and Applications", 3rd Edition, 2017
 *              - Kaplan & Hegarty
 * =======  ========================================================================================
 */

#ifndef STURDR_GNSS_SIGNAL_HPP
#define STURDR_GNSS_SIGNAL_HPP

#include <Eigen/Dense>
#include <complex>

namespace sturdr {

/**
 * *=== CircShift ===
 * @brief Circularly shift a vector
 * @param vec   Vector to shift
 * @param shift Amount to rotate
 * @return Shifted vector
 */
Eigen::VectorXcd CircShift(const Eigen::Ref<const Eigen::VectorXcd> &vec, int shift);

/**
 * *=== AccumulateEPL ===*
 * @brief Accumulates 'n_samp' samples of the current integration period
 * @param rfdata          Recorded signal data
 * @param code            Local code to upsample
 * @param rem_code_phase  Initial fractional phase of the code
 * @param code_freq       GNSS signal code frequency [Hz]
 * @param rem_carr_phase  Initial fractional phase of the carrier [rad]
 * @param carr_freq       Current carrier frequency (including intermediate frequency) [rad/s]
 * @param carr_jit        Current carrier frequency jitter [rad/s^2]
 * @param samp_freq       GNSS receiver front end sampling frequency [Hz]
 * @param half_samp       Number of samples in half the TOTAL accumulation period
 * @param samp_remaining  Number of samples remaining to be accumulated inside TOTAL period
 * @param t_space         Spacing between correlator taps
 * @param E               Early correlator
 * @param P1              Prompt first-half correlator
 * @param P2              Prompt second-half correlator
 * @param L               Late correlator
 */
void AccumulateEPL(
    const Eigen::Ref<const Eigen::VectorXcd> &rfdata,
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
    std::complex<double> &L);
void AccumulateEPLArray(
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
    Eigen::Ref<Eigen::VectorXcd> P1,
    Eigen::Ref<Eigen::VectorXcd> P2,
    std::complex<double> &L);

/**
 * *=== Correlate ===*
 * @brief Correlate a signal to a carrier and code replica
 * @param rfdata  Recorded signal data
 * @param carr    Local carrier replica from NCO
 * @param code    Local code replica from NCO
 * @return GNSS correlator values
 */
std::complex<double> Correlate(
    const Eigen::Ref<const Eigen::VectorXcd> &rfdata,
    const Eigen::Ref<const Eigen::VectorXcd> &carr,
    const Eigen::Ref<const Eigen::VectorXcd> &code);
void Correlate(
    const Eigen::Ref<const Eigen::VectorXcd> &rfdata,
    const bool code[1023],
    double &rem_code_phase,
    double &code_freq,
    double &rem_carr_phase,
    double &carr_freq,
    double &carr_jit,
    double &samp_freq,
    std::complex<double> &C);

/**
 * *=== CodeNCO ===*
 * @brief Creates an upsampled version of the code provided
 * @param code      Local code to upsample
 * @param code_freq GNSS signal code frequency [Hz]
 * @param samp_freq GNSS receiver front end sampling frequency [Hz]
 * @param rem_phase Initial fractional phase of the code
 * @return Upsampled version of provided code
 */
Eigen::VectorXcd CodeNCO(
    const bool code[1023],
    const double &code_freq,
    const double &samp_freq,
    double &rem_phase,
    uint64_t &n_samp);
Eigen::VectorXcd CodeNCO(
    const bool code[1023], const double &code_freq, const double &samp_freq, double &rem_phase);

/**
 * *=== CarrierNCO ===*
 * @brief Creates an sampled version of the carrier wave
 * @param carr_freq Current carrier frequency (including intermediate frequency) [rad/s]
 * @param carr_jit  Current carrier frequency jitter [rad/s^2]
 * @param samp_freq GNSS receiver front end sampling frequency [Hz]
 * @param n_samp    Desired number of samples
 * @param rem_phase Initial fractional phase of the carrier [rad]
 * @return Phase sampled carrier
 */
Eigen::VectorXcd CarrierNCO(
    const double &carr_freq,
    const double &carr_jit,
    const double &samp_freq,
    double &rem_phase,
    const uint64_t &n_samp);

}  // namespace sturdr

#endif