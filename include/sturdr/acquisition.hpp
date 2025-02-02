/**
 * *acquisition.hpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/acquisition.hpp
 * @brief   Satellite acquisition methods.
 * @date    January 2025
 * @author  Daniel Sturdivant <sturdivant20@gmail.com>
 * @ref     1. "Understanding GPS/GNSS Principles and Applications", 3rd Edition, 2017
 *              - Kaplan & Hegarty
 *          2. "A Software-Defined GPS and Galileo Receiver: A Single-Frequency Approach", 2007
 *              - Borre, Akos, Bertelsen, Rinder, Jensen
 * =======  ========================================================================================
 */

// TODO: SerialSearch and Peak2PeakTest

#ifndef STURDR_ACQUISITION_HPP
#define STURDR_ACQUISITION_HPP

#include <Eigen/Dense>
#include <array>

#include "sturdr/fftw-wrapper.hpp"

namespace sturdr {

/**
 * @brief Acquisition NCO replica constants to reduce numerical stress during runtime
 */
struct AcquisitionSetup {
  fftw_plan fft;
  fftw_plan ifft;
  fftw_plan fft_many;
  fftw_plan ifft_many;
  Eigen::MatrixXcd code_fft;
  Eigen::MatrixXcd carr_rep;

  AcquisitionSetup(){};
  AcquisitionSetup(uint64_t rows, uint64_t cols)
      : code_fft{Eigen::MatrixXcd(32, cols)}, carr_rep{Eigen::MatrixXcd(rows, cols)} {};
};

/**
 * @brief initializes known code ffts and carrier replicas
 * @param    codes       Local code (not upsampled)
 * @param    d_range     Max doppler frequency to search [Hz]
 * @param    d_step      Frequency step for doppler search [Z]
 * @param    samp_freq   Front end sampling frequency [Hz]
 * @param    code_freq   GNSS signal code frequency [Hz]
 * @param    intmd_freq  Intermediate frequency of the RF signal [Hz]
 * @param    code_fft
 * @param    carr_rep
 */
AcquisitionSetup InitAcquisitionMatrices(
    const std::array<std::array<bool, 1023>, 32> &codes,
    const double &d_range,
    const double &d_step,
    const double &samp_freq,
    const double &code_freq,
    const double &intmd_freq);

//! === SerialSearch ===

/**
 * *=== PcpsSearch ===*
 * @brief Implementation of the parallel code phase search (PCPS) method
 * @param    rfdata      Data samples recorded by the RF front end
 * @param    code        Local code (not upsampled)
 * @param    d_range     Max doppler frequency to search [Hz]
 * @param    d_step      Frequency step for doppler search [Z]
 * @param    samp_freq   Front end sampling frequency [Hz]
 * @param    code_freq   GNSS signal code frequency [Hz]
 * @param    intmd_freq  Intermediate frequency of the RF signal [Hz]
 * @param    c_per       Number of coherent integrations to perform, by default 1
 * @param    nc_per      Number of non-coherent periods to accumulate, by default 1
 * @return 2D correlation results
 */
Eigen::MatrixXd PcpsSearch(
    const Eigen::VectorXcd &rfdata,
    const uint8_t &c_per,
    const uint8_t &nc_per,
    const uint8_t &prn,
    AcquisitionSetup &acq_setup);
Eigen::MatrixXd PcpsSearch(
    const FftPlans &p,
    const Eigen::VectorXcd &rfdata,
    const std::array<bool, 1023> &code,
    const double &d_range,
    const double &d_step,
    const double &samp_freq,
    const double &code_freq,
    const double &intmd_freq,
    const uint8_t &c_per,
    const uint8_t &nc_per);

//! === Peak2PeakTest ===

/**
 * *=== Peak2NoiseFloorTest ===*
 * @brief Compares the two highest peak to the noise floor of the acquisition plane
 * @param corr_map       2D-array from correlation method
 * @param peak_idx       Indexes of highest correlation peak
 * @param metric         Ratio between the highest and second highest peaks
 */
void Peak2NoiseFloorTest(const Eigen::MatrixXd &corr_map, int peak_idx[2], double &metric);

/**
 * *=== GlrtTest ===*
 * @brief General likelihood ratio test
 * @param corr_map       2D-array from correlation method
 * @param peak_idx       Indexes of highest correlation peak
 * @param metric         Ratio between the highest and second highest peaks
 */
void GlrtTest(
    const Eigen::MatrixXd &corr_map,
    int peak_idx[2],
    double &metric,
    const Eigen::VectorXcd &rfdata);

}  // end namespace sturdr

#endif