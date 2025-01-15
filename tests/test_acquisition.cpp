
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <Eigen/Dense>
#include <array>
#include <complex>
#include <fstream>
#include <satutils/code-gen.hpp>

#include "sturdr/acquisition.hpp"

int main() {
  // Create spdlog multi-threaded console/terminal logger
  std::shared_ptr<spdlog::logger> console = spdlog::stdout_color_mt("sturdr-console");
  console->set_pattern("\033[1;34m[%D %T.%e][%^%l%$\033[1;34m]: \033[0m%v");
  console->set_level(spdlog::level::debug);
  console->debug("test_acquisition.cpp initializing!");

  // Initialize
  double samp_freq = 20e6;
  double intmd_freq = 5000445.88565834;
  double code_freq = 1.023e6;
  double threshold = 12.0;
  double d_range = 5000.0;
  double d_step = 100.0;
  uint8_t c_per = 1;
  uint8_t nc_per = 5;
  // double carr_freq = 1575.42e6;
  // bool test_p2n = true;

  // create fft plans (shared across all threads)
  int samp_per_ms = static_cast<int>(samp_freq / 1000.0);
  int n_dopp_bins = static_cast<int>(2.0 * d_range / d_step + 1.0);
  sturdr::SturdrFftPlans p;
  p.fft = sturdr::Create1dFftPlan(samp_per_ms, true);
  p.ifft = sturdr::Create1dFftPlan(samp_per_ms, false);
  p.fft_many = sturdr::CreateManyFftPlan(n_dopp_bins, samp_per_ms, true);
  p.ifft_many = sturdr::CreateManyFftPlan(n_dopp_bins, samp_per_ms, false);
  console->debug("FFT plans created!");

  // load signal
  uint64_t n_samp = static_cast<uint64_t>(c_per * nc_per * static_cast<int>(samp_freq) / 1000);
  std::ifstream file(
      "../sturdr/rfdata/class_ifen_8bit_20e6_if_5000445.88565834.bin", std::ios::binary);
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
  std::array<bool, 1023> code;
  satutils::CodeGenCA(code, 1);
  console->debug("C/A code generated!");

  // run acquisition
  Eigen::MatrixXd corr_map = sturdr::PcpsSearch(
      p, signal, code, d_range, d_step, samp_freq, code_freq, intmd_freq, c_per, nc_per);
  int peak_idx[2];
  double metric;
  sturdr::Peak2NoiseFloorTest(corr_map, peak_idx, metric);
  double doppler = -d_range + static_cast<double>(peak_idx[0]) * d_step;
  console->info("Doppler: {:.1f} Hz", doppler);
  console->info("Code Phase: {:d} Samples", peak_idx[1]);
  console->info("Metric: {:.1f}", metric);

  // Did we acquire
  if (metric > threshold) {
    console->info("EUREKA! It worked!");
  }

  file.close();
  return 0;
}