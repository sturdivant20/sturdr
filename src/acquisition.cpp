/**
 * *acquisition.cpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/acquisition.cpp
 * @brief   Satellite acquisition methods.
 * @date    December 2024
 * @author  Daniel Sturdivant <sturdivant20@gmail.com>
 * @ref     1. "Understanding GPS/GNSS Principles and Applications", 3rd Edition, 2017
 *              - Kaplan & Hegarty
 *          2. "A Software-Defined GPS and Galileo Receiver: A Single-Frequency Approach", 2007
 *              - Borre, Akos, Bertelsen, Rinder, Jensen
 * =======  ========================================================================================
 */

#include "sturdr/acquisition.hpp"

#include <spdlog/spdlog.h>

#include <cmath>
#include <exception>
#include <navtools/constants.hpp>

#include "sturdr/gnss-signal.hpp"

namespace sturdr {

// *=== PcpsSearch ===*
Eigen::MatrixXd PcpsSearch(
    const SturdrFftPlans &p,
    const Eigen::VectorXcd &rfdata,
    const std::array<bool, 1023> &code,
    const double &d_range,
    const double &d_step,
    const double &samp_freq,
    const double &code_freq,
    const double &intmd_freq,
    const uint8_t &c_per,
    const uint8_t &nc_per) {
  try {
    // Doppler bins
    uint64_t n_dopp_bins = static_cast<uint64_t>(2.0 * d_range / d_step + 1.0);
    Eigen::VectorXd dopp_bins =
        Eigen::VectorXd::LinSpaced(n_dopp_bins, -d_range, d_range).array() + intmd_freq;

    // Initialize code replica
    double rem_phase = 0.0;
    Eigen::VectorXcd code_up = CodeNCO(code, code_freq, samp_freq, rem_phase);
    uint64_t samp_per_code = code_up.size();

    // initialize carrier replica
    Eigen::VectorXd carr_phase_pts =
        Eigen::VectorXd::LinSpaced(samp_per_code, 0.0, static_cast<double>(samp_per_code - 1)) *
        (navtools::TWO_PI<double> / samp_freq);
    Eigen::MatrixXcd carr_up =
        -navtools::COMPLEX_I<double> * (dopp_bins * carr_phase_pts.transpose());
    carr_up = carr_up.array().exp();

    // Allocate correlation results map
    Eigen::MatrixXd corr_map = Eigen::MatrixXd::Zero(n_dopp_bins, samp_per_code);
    Eigen::MatrixXcd coh_sum = Eigen::MatrixXcd::Zero(n_dopp_bins, samp_per_code);
    Eigen::MatrixXcd x_carr = Eigen::MatrixXcd::Zero(n_dopp_bins, samp_per_code);

    // initialize FFT plans
    // ThreadSafety();
    // fftw_plan fft_code = Create1dFftPlan(code_up, code_up, samp_per_code, true);
    // fftw_plan fft = Create2dFftPlan(x_carr, x_carr, n_dopp_bins, samp_per_code, true);
    // fftw_plan ifft = Create2dFftPlan(x_carr, x_carr, n_dopp_bins, samp_per_code, false);

    // Perform code FFT ahead of time (conjugate result)
    ExecuteFftPlan(p.fft, code_up, code_up);
    code_up = code_up.conjugate();

    // Loop through each non-coherent period
    uint64_t i_sig = 0;
    for (uint8_t i_nc = 0; i_nc < nc_per; i_nc++) {
      coh_sum = Eigen::MatrixXcd::Zero(n_dopp_bins, samp_per_code);

      // Loop through each coherent period
      for (uint8_t j_c = 0; j_c < c_per; j_c++) {
        // Wiped carrier FFT
        x_carr =
            carr_up.array().rowwise() * rfdata.segment(i_sig, samp_per_code).array().transpose();
        ExecuteManyFftPlan(p.fft_many, x_carr, x_carr);

        // Combined Code-Wiped Carrier IFFT
        x_carr = x_carr.array().rowwise() * code_up.array().transpose();
        ExecuteManyFftPlan(p.ifft_many, x_carr, x_carr);

        // coherent sum
        coh_sum += x_carr;  // x_carr.array() / n ??
        i_sig += samp_per_code;
      }

      // sum power noncoherently
      corr_map += coh_sum.cwiseAbs2();
    }

    // fftw_destroy_plan(fft_code);
    // fftw_destroy_plan(fft);
    // fftw_destroy_plan(ifft);
    return corr_map;
  } catch (std::exception &e) {
    spdlog::get("sturdr-console")
        ->error("acquisition.cpp PcpsSearch failed! Error -> {}", e.what());
    Eigen::VectorXd tmp;
    return tmp;
  }
}

// *=== Peak2NoiseFloorTest ===*
void Peak2NoiseFloorTest(const Eigen::MatrixXd &corr_map, int peak_idx[2], double &metric) {
  try {
    // find the highest correlation peak
    double peak = corr_map.maxCoeff(&peak_idx[0], &peak_idx[1]);

    // calculate mean and covariance
    double mu = corr_map.mean();
    double sigma = std::sqrt((corr_map.array() - mu).square().sum() / (corr_map.size() - 1));

    // // One pass approximation
    // double peak = 0.0;
    // double s = 0.0;
    // double s2 = 0.0;
    // double n = static_cast<double>(corr_map.size());
    // for (int i = 0; i < corr_map.rows(); i++) {
    //   for (int j = 0; j < corr_map.cols(); j++) {
    //     s += corr_map(i, j);
    //     s2 += (corr_map(i, j) * corr_map(i, j));

    //     // test for new peak
    //     if (peak < corr_map(i, j)) {
    //       peak = corr_map(i, j);
    //       peak_idx[0] = i;
    //       peak_idx[1] = j;
    //     }
    //   }
    // }
    // double mu = s / n;
    // double sigma = std::sqrt(s2 / n - mu * mu);  //(s2 - (s * s / n)) / n;

    // calculate acquisition metric
    metric = (peak - mu) / sigma;
  } catch (std::exception &e) {
    spdlog::get("sturdr-console")
        ->error("acquisition.cpp Peak2NoiseFloorTest failed! Error -> {}", e.what());
  }
}

}  // end namespace sturdr