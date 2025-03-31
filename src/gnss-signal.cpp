/**
 * *gnss-signal.cpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/gnss-signal.cpp
 * @brief   Common GNSS signal functions.
 * @date    January 2025
 * @author  Daniel Sturdivant <sturdivant20@gmail.com>
 * @ref     1. "A Software-Defined GPS and Galileo Receiver: A Single-Frequency Approach", 2007
 *              - Borre, Akos, Bertelsen, Rinder, Jensen
 *          2. "Understanding GPS/GNSS Principles and Applications", 3rd Edition, 2017
 *              - Kaplan & Hegarty
 * =======  ========================================================================================
 */

#include "sturdr/gnss-signal.hpp"

#include <cmath>
#include <navtools/constants.hpp>

namespace sturdr {

// *=== CircShift ===*
Eigen::VectorXcd CircShift(const Eigen::Ref<const Eigen::VectorXcd> &vec, int shift) {
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

// *=== AccumulateEPL ===*
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
    std::complex<double> &L) {
  // init phase increments
  double d_code = code_freq / samp_freq;
  double d_carr = (carr_freq + 0.5 * carr_jit / samp_freq) / samp_freq;

  // loop through number of samples
  std::complex<double> v_carr;
  double v_code;
  for (const std::complex<double> &sample : rfdata) {
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
    std::complex<double> E,
    Eigen::Ref<Eigen::VectorXcd> P1,
    Eigen::Ref<Eigen::VectorXcd> P2,
    std::complex<double> L) {
  // init phase increments
  double d_code = code_freq / samp_freq;
  double d_carr = (carr_freq + 0.5 * carr_jit / samp_freq) / samp_freq;

  // loop through number of samples
  int n_samp = rfdata.rows();
  Eigen::VectorXcd v_carr;
  double v_code;
  for (int ii = 0; ii < n_samp; ii++) {
    v_carr = std::exp(-navtools::COMPLEX_I<> * rem_carr_phase) * rfdata.row(ii);

    // early
    // v_code = code[static_cast<int>(std::fmod(rem_code_phase + t_space, 1023.0))] ? 1.0 : -1.0;
    v_code = code[static_cast<int>(std::round(rem_code_phase + t_space)) % 1023] ? 1.0 : -1.0;
    E += (v_code * v_carr(0));

    // late
    // v_code = code[static_cast<int>(std::fmod(rem_code_phase - t_space, 1023.0))] ? 1.0 : -1.0;
    v_code = code[static_cast<int>(std::round(rem_code_phase - t_space)) % 1023] ? 1.0 : -1.0;
    L += (v_code * v_carr(0));

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

//* === Correlate ===*
std::complex<double> Correlate(
    const Eigen::Ref<const Eigen::VectorXcd> &rfdata,
    const Eigen::Ref<const Eigen::VectorXcd> &carr,
    const Eigen::Ref<const Eigen::VectorXcd> &code) {
  return (code.array() * carr.array() * rfdata.array()).sum();
}
void Correlate(
    const Eigen::Ref<const Eigen::VectorXcd> &rfdata,
    const bool code[1023],
    double &rem_code_phase,
    double &code_freq,
    double &rem_carr_phase,
    double &carr_freq,
    double &carr_jit,
    double &samp_freq,
    std::complex<double> &C) {
  // init phase increments
  double d_code = code_freq / samp_freq;
  double d_carr = (carr_freq + 0.5 * carr_jit / samp_freq) / samp_freq;

  // loop through number of samples
  std::complex<double> v_carr;
  double v_code;
  for (const std::complex<double> &sample : rfdata) {
    v_carr = std::exp(-navtools::COMPLEX_I<> * rem_carr_phase) * sample;
    v_code = code[static_cast<int>(std::round(rem_code_phase)) % 1023] ? 1.0 : -1.0;
    C += (v_code * v_carr);

    // increment
    rem_code_phase += d_code;
    rem_carr_phase += d_carr;
  }
}

// *=== CodeNCO ===*
Eigen::VectorXcd CodeNCO(
    const bool code[1023],
    const double &code_freq,
    const double &samp_freq,
    double &rem_phase,
    uint64_t &n_samp) {
  // init
  Eigen::VectorXd code_up(n_samp);
  double d_code = code_freq / samp_freq;

  // upsample
  for (uint64_t i = 0; i < n_samp; i++) {
    code_up(i) = code[static_cast<uint64_t>(std::fmod(rem_phase, 1023.0))] ? 1.0 : -1.0;
    rem_phase += d_code;
  }

  return code_up;
}
Eigen::VectorXcd CodeNCO(
    const bool code[1023], const double &code_freq, const double &samp_freq, double &rem_phase) {
  uint64_t n_samp = static_cast<uint64_t>(std::ceil((1023.0 - rem_phase) / code_freq * samp_freq));
  return CodeNCO(code, code_freq, samp_freq, rem_phase, n_samp);
}

// *=== CarrierNCO ===*
Eigen::VectorXcd CarrierNCO(
    const double &carr_freq,
    const double &carr_jit,
    const double &samp_freq,
    double &rem_phase,
    const uint64_t &n_samp) {
  // init
  Eigen::VectorXcd carr_up(n_samp);
  double d_carr = (carr_freq + 0.5 * carr_jit / samp_freq) / samp_freq;

  // upsample
  for (uint64_t i = 0; i < n_samp; i++) {
    carr_up(i) = std::exp(-navtools::COMPLEX_I<> * rem_phase);
    rem_phase += d_carr;
  }

  return carr_up;
}

}  // namespace sturdr