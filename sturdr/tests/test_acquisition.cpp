
#include <Eigen/Dense>
#include <array>
#include <complex>
#include <fstream>
#include <iostream>

#include "sturdr/channel/gps-l1ca-channel.hpp"
#include "sturdr/dsp/acquisition.hpp"

int main() {
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

  // load signal
  uint64_t n_samp = static_cast<uint64_t>(c_per * nc_per * static_cast<int>(samp_freq) / 1000);
  std::ifstream file(
      "../sturdr/rfdata/class_ifen_8bit_20e6_if_5000445.88565834.bin", std::ios::binary);
  if (!file.is_open()) {
    std::cerr << "Error opening file!" << std::endl;
    return 1;
  }

  char byte;
  Eigen::VectorXcd signal(n_samp);
  for (uint64_t i = 0; i < n_samp; i++) {
    file.read(&byte, 1);
    signal(i) = static_cast<std::complex<double>>(byte);
    // std::cout << signal(i) << "\n";
  }

  // generate ca code
  std::array<bool, 1023> code;
  sturdr::CodeGenCA(code, 7);

  // run acquisition
  Eigen::MatrixXd corr_map = sturdr::PcpsSearch(
      signal, code, d_range, d_step, samp_freq, code_freq, intmd_freq, c_per, nc_per);
  int peak_idx[2];
  double metric;
  sturdr::Peak2NoiseFloorTest(corr_map, peak_idx, metric);

  // Did we acquire
  if (metric > threshold) {
    std::cout << "\nEUREKA! It worked! \n";
    double doppler = -d_range + static_cast<double>(peak_idx[0]) * d_step;
    std::cout << "Doppler: " << doppler << " Hz\n";
    std::cout << "Code Phase: " << peak_idx[1] << " Samples\n";
    std::cout << "Metric: " << metric << "\n";
  }

  file.close();
  return 0;
}