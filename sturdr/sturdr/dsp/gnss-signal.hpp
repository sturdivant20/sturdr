/**
 * *gnss-signal.hpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/dsp/gnss-signal.hpp
 * @brief   Common GNSS signal functions.
 * @date    December 2024
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
#include <array>

namespace sturdr {

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
    std::array<bool, 1023> &code, double &code_freq, double &samp_freq, double &rem_phase);

/**
 * *=== CarrierNCO ===*
 * @brief Creates an upsampled version of the code provided
 * @param carr_freq Current carrier frequency (including intermediate frequency) [rad/s]
 * @param carr_jit  Current carrier frequency jitter [rad/s^2]
 * @param samp_freq GNSS receiver front end sampling frequency [Hz]
 * @param n_samp    Desired number of samples
 * @param rem_phase Initial fractional phase of the carrier [rad]
 * @return Upsampled version of provided code
 */
Eigen::VectorXcd CarrierNCO(
    double &carr_freq, double &carr_jit, double &samp_freq, uint64_t &n_samp, double &rem_phase);

/**
 * *=== Correlate ===*
 * @brief Correlate a signal to a carrier and code replica
 * @param rfdata  Recorded signal data
 * @param carr    Local carrier replica from NCO
 * @param code    Local code replica from NCO
 * @return GNSS correlator values
 */
std::complex<double> Correlate(
    const Eigen::VectorXcd &rfdata, const Eigen::VectorXcd &carr, const Eigen::VectorXcd &code);

/**
 * *=== AccumulateEPL ===*
 * @brief Accumulates 'n_samp' samples of the current integration period
 * @param rfdata  Recorded signal data
 * @param code            Local code to upsample
 * @param rem_code_phase  Initial fractional phase of the code
 * @param code_freq       GNSS signal code frequency [Hz]
 * @param rem_carr_phase  Initial fractional phase of the carrier [rad]
 * @param carr_freq       Current carrier frequency (including intermediate frequency) [rad/s]
 * @param carr_jit        Current carrier frequency jitter [rad/s^2]
 * @param samp_freq       GNSS receiver front end sampling frequency [Hz]
 * @param n_samp          Desired number of samples
 * @param samp_cnt        Current number of samples accumulated inside TOTAL accumulation period
 * @param half_samp       Number of samples in half the TOTAL accumulation period
 * @param tap_spacing     Spacing between correlator taps
 * @param E               Early correlator
 * @param P1              Prompt first-half correlator
 * @param P2              Prompt second-half correlator
 * @param L               Late correlator
 */
void AccumulateEPL(
    const Eigen::VectorXcd &rfdata,
    std::array<bool, 1023> &code,
    double &rem_code_phase,
    double &code_freq,
    double &rem_carr_phase,
    double &carr_freq,
    double &carr_jit,
    double &samp_freq,
    uint64_t &n_samp,
    uint64_t &samp_cnt,
    uint64_t &half_samp,
    double &tap_spacing,
    std::complex<double> &E,
    std::complex<double> &P1,
    std::complex<double> &P2,
    std::complex<double> &L);

/**
 * *=== CircShift ===
 * @brief Circularly shift a vector
 * @param vec   Vector to shift
 * @param shift Amount to rotate
 * @return Shifted vector
 */
Eigen::VectorXcd CircShift(const Eigen::VectorXcd &vec, int shift);

}  // end namespace sturdr

#endif