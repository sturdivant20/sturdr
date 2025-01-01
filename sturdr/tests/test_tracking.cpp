
#include <array>
#include <fstream>
#include <iostream>
#include <navtools/constants.hpp>

#include "sturdr/channel/gps-l1ca-channel.hpp"
#include "sturdr/dsp/discriminator.hpp"
#include "sturdr/dsp/gnss-signal.hpp"
#include "sturdr/dsp/lock-detectors.hpp"
#include "sturdr/dsp/tracking.hpp"

bool MODE = 1;  // 0 - Loop Filters, 1 - Kalman Filter

int main() {
  // config
  double samp_freq = 20e6;
  double intmd_freq = navtools::TWO_PI<double> * 5000445.88565834;
  double code_freq = 1.023e6;
  double carr_freq = navtools::TWO_PI<double> * 1575.42e6;
  double code_len = 1023.0;
  double k = code_freq / carr_freq;

  // acquisition handover
  std::array<bool, 1023> code;
  sturdr::CodeGenCA(code, 1);
  double doppler = navtools::TWO_PI<double> * 1800.0;
  double jitter = 0.0;
  int code_samples = 17414;
  double code_doppler = doppler * k;
  // sturdr::CodeGenCA(code, 7);
  // double doppler = navtools::TWO_PI<double> * -500.0;
  // int code_samples = 19325;
  double cno = 1000.0;

  // open file
  std::ifstream file(
      "../sturdr/rfdata/class_ifen_8bit_20e6_if_5000445.88565834.bin", std::ios::binary);
  if (!file.is_open()) {
    std::cerr << "Error opening file!" << std::endl;
    return 1;
  }
  file.seekg(code_samples, std::ios::beg);

  // initialize tracking filters
  int tap_spacing = static_cast<int>(0.25 * samp_freq / code_freq);
  double w0p = sturdr::NaturalFrequency(15.0, 3);
  double w0f = sturdr::NaturalFrequency(2.5, 2);
  double w0d = sturdr::NaturalFrequency(1.0, 2);
  double jit_accum = 0.0;
  double dop_accum = doppler;
  double chp_accum = 0.0;
  double NBD = 0.0;
  double NBP = 0.0;
  double PC = 0.0;
  double PN = 0.0;
  std::complex<double> E{0.0, 0.0};
  std::complex<double> P{0.0, 0.0};
  std::complex<double> L{0.0, 0.0};
  std::complex<double> P1{0.0, 0.0};
  std::complex<double> P2{0.0, 0.0};
  std::complex<double> P_prev{0.0, 0.0};
  double rem_carr_phase = 0.0;
  double rem_code_phase = 0.0;
  double phase_err, freq_err, chip_err;
  bool code_lock, carr_lock;
  double T = 0.001, t = T / 2.0;
  sturdr::TrackingKF kf;
  kf.Init(w0d, w0p, w0f, k, rem_carr_phase, doppler, rem_code_phase, cno, T, intmd_freq, code_freq);

  // run tracking
  double carr_freq_nco = intmd_freq + doppler;
  double code_freq_nco = code_freq;
  Eigen::VectorXcd code_prompt, code_early, code_late, carr, rfdata;
  uint64_t n_samp, half_samp;
  for (uint64_t i = 0; i < 1000; i++) {
    // upsample code
    code_freq_nco = code_freq + code_doppler;
    code_prompt = sturdr::CodeNCO(code, code_freq_nco, samp_freq, rem_code_phase);
    code_early = sturdr::CircShift(code_prompt, +tap_spacing);
    code_late = sturdr::CircShift(code_prompt, -tap_spacing);

    // sample count
    n_samp = static_cast<uint64_t>(code_prompt.size());
    half_samp = static_cast<uint64_t>(n_samp / 2);
    T = static_cast<double>(n_samp) / samp_freq;
    t = T / 2.0;

    // upsample carrier
    carr_freq_nco = intmd_freq + doppler;
    carr = sturdr::CarrierNCO(carr_freq_nco, jitter, samp_freq, n_samp, rem_carr_phase);

    // read signal data
    rfdata.resize(n_samp);
    char byte;
    for (uint64_t k = 0; k < n_samp; k++) {
      file.read(&byte, 1);
      rfdata(k) = static_cast<std::complex<double>>(byte);
    }

    // correlate data
    E = sturdr::Correlate(rfdata, carr, code_early);
    P1 = sturdr::Correlate(
        rfdata.head(half_samp), carr.head(half_samp), code_prompt.head(half_samp));
    P2 = sturdr::Correlate(
        rfdata.tail(n_samp - half_samp),
        carr.tail(n_samp - half_samp),
        code_prompt.tail(n_samp - half_samp));
    L = sturdr::Correlate(rfdata, carr, code_late);
    P = P1 + P2;

    // discriminators
    phase_err = sturdr::PllCostas(P);
    freq_err = sturdr::FllAtan2(P1, P2, t);
    chip_err = sturdr::DllNneml2(E, L);

    // lock detectors
    sturdr::LockDetectors(code_lock, carr_lock, cno, NBD, NBP, PC, PN, P_prev, P, T);

    // filter
    if (MODE) {
      kf.UpdateDynamicsParam(w0d, w0p, w0f, k, T);
      kf.UpdateMeasurementsParam(cno, T);
      Eigen::Vector<double, 5> x = kf.Run(chip_err, phase_err, freq_err);
      rem_carr_phase = std::fmod(x(0), navtools::TWO_PI<double>);
      doppler = x(1);
      jitter = x(2);
      rem_code_phase = x(3) - code_len;
      code_doppler = x(4);
      kf.SetRemCarrierPhase(rem_carr_phase);
      kf.SetRemCodePhase(rem_code_phase);
    } else {
      sturdr::FLLassistedPLL_3rdOrder(
          doppler, dop_accum, jit_accum, phase_err, freq_err, T, w0p, w0f);
      sturdr::PLL_2ndOrder(code_doppler, chp_accum, chip_err, T, w0d);
      code_doppler += doppler * k;
    }
    P_prev = P;

    // save results

    std::cout << "------------------------------\n";
    std::cout << "i: " << i << ", n_samp: " << n_samp << "\n";
    std::cout << "IE: " << E.real() << ", IP: " << P.real() << ", IL: " << L.real() << "\n";
    std::cout << "QE: " << E.imag() << ", QP: " << P.imag() << ", QL: " << L.imag() << "\n";
    std::cout << "phase_err: " << phase_err << ", freq_err: " << freq_err
              << ", chip_err: " << chip_err << "\n";
    std::cout << "cno: " << 10.0 * std::log10(cno) << ", code_lock: " << code_lock
              << ", carr_lock: " << carr_lock << "\n";
    std::cout << "code_doppler: " << code_doppler << ", rem_code_phase: " << rem_code_phase << "\n";
    std::cout << "carr_jitter: " << jitter / navtools::TWO_PI<double> << ", carr_doppler: "
              << doppler / navtools::TWO_PI<double> << ", rem_carr_phase: " << rem_carr_phase
              << "\n\n";
  }

  return 0;
}