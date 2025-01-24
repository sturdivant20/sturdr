
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <Eigen/Dense>
#include <array>
#include <complex>
#include <fstream>
#include <satutils/code-gen.hpp>
#include <satutils/gnss-constants.hpp>

#include "sturdr/acquisition.hpp"
#include "sturdr/fftw-wrapper.hpp"

int main() {
  // Create spdlog multi-threaded console/terminal logger
  std::shared_ptr<spdlog::logger> console = spdlog::stdout_color_mt("sturdr-console");
  console->set_pattern("\033[1;34m[%D %T.%e][%^%l%$\033[1;34m]: \033[0m%v");
  console->set_level(spdlog::level::debug);
  console->debug("test_acquisition.cpp initializing!");

  // Initialize
  double samp_freq = 20e6;
  double intmd_freq = 5000445.88565834;
  double threshold = 12.0;
  double d_range = 5000.0;
  double d_step = 100.0;
  uint8_t c_per = 1;
  uint8_t nc_per = 5;
  // double carr_freq = 1575.42e6;
  // bool test_p2n = true;

  // initialize acquisition matrices
  std::array<std::array<bool, 1023>, 32> codes;
  for (int i = 0; i < 32; i++) {
    satutils::CodeGenCA(codes[i], i + 1);
  }
  sturdr::AcquisitionSetup acq_setup = sturdr::InitAcquisitionMatrices(
      codes, d_range, d_step, samp_freq, satutils::GPS_CA_CODE_RATE<>, intmd_freq);
  int n_dopp_bins = 2 * static_cast<int>(d_range / d_step) + 1;
  int samp_per_ms = static_cast<int>(samp_freq) / 1000;
  sturdr::FftPlans plans;
  // plans.fft = sturdr::Create1dFftPlan(samp_per_ms, true),
  // plans.ifft = sturdr::Create1dFftPlan(samp_per_ms, false),
  // plans.fft_many = sturdr::CreateManyFftPlan(n_dopp_bins, samp_per_ms, true),
  // plans.ifft_many = sturdr::CreateManyFftPlan(n_dopp_bins, samp_per_ms, false);
  plans.fft = sturdr::Create1dFftPlan(samp_per_ms, true),
  plans.ifft = sturdr::Create1dFftPlan(samp_per_ms, false),
  plans.fft_many = sturdr::CreateManyFftPlanColWise(samp_per_ms, n_dopp_bins, true),
  plans.ifft_many = sturdr::CreateManyFftPlanColWise(samp_per_ms, n_dopp_bins, false);

  console->debug("FFT plans created!");

  // load signal
  uint64_t n_samp = static_cast<uint64_t>(c_per * nc_per * static_cast<int>(samp_freq) / 1000);
  std::ifstream file("data/gpsBase_IFEN_IF.bin", std::ios::binary);
  if (!file.is_open()) {
    console->error("Error opening file!");
    return 1;
  }

  char byte;
  Eigen::VectorXcd signal(n_samp);
  for (uint64_t i = 0; i < n_samp; i++) {
    file.read(&byte, 1);
    signal(i) = static_cast<std::complex<double>>(byte);
  }
  console->debug("RF signal read from file!");

  // generate ca code
  uint8_t prn = 1;
  std::array<bool, 1023> code;
  satutils::CodeGenCA(code, prn);
  console->debug("C/A code generated!");

  // run acquisition
  // Eigen::MatrixXd corr_map = sturdr::PcpsSearch(
  //     p, signal, code, d_range, d_step, samp_freq, code_freq, intmd_freq, c_per, nc_per);
  Eigen::MatrixXd corr_map = PcpsSearch(signal, c_per, nc_per, prn, acq_setup);
  int peak_idx[2];
  double metric;
  sturdr::Peak2NoiseFloorTest(corr_map, peak_idx, metric);
  double doppler = -d_range + static_cast<double>(peak_idx[0]) * d_step;
  console->info("Doppler: {:.1f} Hz", doppler);
  console->info("Code Phase: {:d} Samples", peak_idx[1]);
  console->info("Metric: {:.1f}", metric);

  Eigen::MatrixXd corr_map2 = PcpsSearch(
      plans,
      signal,
      code,
      d_range,
      d_step,
      samp_freq,
      satutils::GPS_CA_CODE_RATE<>,
      intmd_freq,
      c_per,
      nc_per);
  int peak_idx2[2];
  double metric2;
  sturdr::Peak2NoiseFloorTest(corr_map2, peak_idx2, metric2);
  double doppler2 = -d_range + static_cast<double>(peak_idx2[1]) * d_step;
  console->info("Doppler2: {:.1f} Hz", doppler2);
  console->info("Code Phase2: {:d} Samples", peak_idx2[0]);
  console->info("Metric2: {:.1f}", metric2);

  // Did we acquire
  if (metric > threshold) {
    console->info("EUREKA! It worked!");
  }

  file.close();
  return 0;
}