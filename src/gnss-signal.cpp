/**
 * *gnss-signal.cpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/gnss-signal.cpp
 * @brief   Common GNSS signal functions.
 * @date    December 2024
 * @author  Daniel Sturdivant <sturdivant20@gmail.com>
 * @ref     1. "A Software-Defined GPS and Galileo Receiver: A Single-Frequency Approach", 2007
 *              - Borre, Akos, Bertelsen, Rinder, Jensen
 *          2. "Understanding GPS/GNSS Principles and Applications", 3rd Edition, 2017
 *              - Kaplan & Hegarty
 * =======  ========================================================================================
 */

#include "sturdr/gnss-signal.hpp"

#include <spdlog/spdlog.h>

#include <cmath>
#include <complex>
#include <exception>
#include <navtools/constants.hpp>

namespace sturdr {

// *=== CodeNCO ===*
Eigen::VectorXcd CodeNCO(
    const std::array<bool, 1023> &code,
    const double &code_freq,
    const double &samp_freq,
    double &rem_phase) {
  try {
    // Initialize
    double N = static_cast<double>(code.size());
    double code_phase_step = code_freq / samp_freq;
    uint64_t samp_per_code = static_cast<uint64_t>(std::ceil((N - rem_phase) / code_phase_step));

    // Upsample code
    std::complex<double> p1(1.0, 0.0);
    std::complex<double> m1(-1.0, 0.0);
    Eigen::VectorXcd code_up(samp_per_code);
    for (uint64_t i = 0; i < samp_per_code; i++) {
      code_up(i) = code[static_cast<uint64_t>(std::fmod(rem_phase, N))] ? p1 : m1;
      rem_phase += code_phase_step;
    }
    rem_phase -= N;
    return code_up;

  } catch (std::exception &e) {
    spdlog::get("sturdr-console")->error("gnss-signal.cpp CodeNCO failed! Error -> {}", e.what());
    Eigen::VectorXcd tmp;
    return tmp;
  }
}

// *=== CarrierNCO ===*
Eigen::VectorXcd CarrierNCO(
    const double &carr_freq,
    const double &carr_jit,
    const double &samp_freq,
    const uint64_t &n_samp,
    double &rem_phase) {
  try {
    // Initialize
    Eigen::VectorXcd carr_up(n_samp);
    double carr_phase_step = (carr_freq + 0.5 * carr_jit / samp_freq) / samp_freq;

    // Discretize carrier
    for (uint64_t i = 0; i < n_samp; i++) {
      carr_up(i) = std::exp(-navtools::COMPLEX_I<double> * rem_phase);
      rem_phase += carr_phase_step;
    }
    rem_phase = std::fmod(rem_phase, navtools::TWO_PI<double>);
    return carr_up;

  } catch (std::exception &e) {
    spdlog::get("sturdr-console")
        ->error("gnss-signal.cpp CarrierNCO failed! Error -> {}", e.what());
    Eigen::VectorXcd tmp;
    return tmp;
  }
}

// *=== Correlate ===*
std::complex<double> Correlate(
    const Eigen::VectorXcd &rfdata, const Eigen::VectorXcd &carr, const Eigen::VectorXcd &code) {
  try {
    return (rfdata.array() * carr.array() * code.array()).sum();

  } catch (std::exception &e) {
    spdlog::get("sturdr-console")->error("gnss-signal.cpp Correlate failed! Error -> {}", e.what());
    double n = std::nan("1");
    return std::complex<double>(n, n);
  }
}

// *=== AccumulateEPL ===*
void AccumulateEPL(
    const Eigen::VectorXcd &rfdata,
    std::array<bool, 1023> &code,
    double &rem_code_phase,
    const double &code_freq,
    double &rem_carr_phase,
    const double &carr_freq,
    const double &carr_jit,
    const double &samp_freq,
    const uint64_t &n_samp,
    uint64_t &samp_cnt,
    const uint64_t &half_samp,
    const double &tap_spacing,
    std::complex<double> &E,
    std::complex<double> &P1,
    std::complex<double> &P2,
    std::complex<double> &L) {
  try {
    // Initialize
    double N = static_cast<double>(code.size());
    double code_phase_step = code_freq / samp_freq;
    double carr_phase_step = (carr_freq + 0.5 * carr_jit / samp_freq) / samp_freq;

    // Accumulate
    std::complex<double> x_carr;
    for (uint64_t i = 0; i < n_samp; i++) {
      // Wipe carrier
      x_carr = std::exp(-navtools::COMPLEX_I<double> * rem_carr_phase) * rfdata(i);

      // Correlate
      E += x_carr *
           (code[static_cast<uint64_t>(std::fmod(rem_code_phase + tap_spacing, N))] ? 1.0 : -1.0);
      if (samp_cnt < half_samp) {
        P1 += x_carr * (code[static_cast<uint64_t>(std::fmod(rem_code_phase, N))] ? 1.0 : -1.0);
      } else {
        P2 += x_carr * (code[static_cast<uint64_t>(std::fmod(rem_code_phase, N))] ? 1.0 : -1.0);
      }
      L += x_carr *
           (code[static_cast<uint64_t>(std::fmod(rem_code_phase - tap_spacing, N))] ? 1.0 : -1.0);

      // propagate
      samp_cnt += 1;
      rem_code_phase += code_phase_step;
      rem_carr_phase += carr_phase_step;
    }

  } catch (std::exception &e) {
    spdlog::get("sturdr-console")
        ->error("gnss-signal.cpp AccumulateEPL failed! Error -> {}", e.what());
  }
}

// *=== CircShift ===
Eigen::VectorXcd CircShift(const Eigen::VectorXcd &vec, int shift) {
  int n = vec.size();
  shift = shift % n;  // Ensure shift is within bounds

  if (shift == 0) {
    return vec;
  }

  Eigen::VectorXcd shifted(n);

  if (shift > 0) {
    shifted << vec.tail(n - shift), vec.head(shift);
  } else {
    shift = -shift;
    shifted << vec.tail(shift), vec.head(n - shift);
  }

  return shifted;
}

}  // end namespace sturdr