/**
 * *fftw-wrapper.hpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/utils/fftw-wrapper.hpp
 * @brief   Small functions for using complex-1d FFTW3 methods specific to STURDR.
 * @date    December 2024
 * @author  Daniel Sturdivant <sturdivant20@gmail.com>
 * =======  ========================================================================================
 */

#ifndef STURDR_FFTW_WRAPPER_HPP
#define STURDR_FFTW_WRAPPER_HPP

#include <fftw3.h>

#include <Eigen/Dense>

namespace sturdr {

//! === ThreadSafety ===
/// @brief Ensures the creation of FFT planners does not conflict with other processes
void ThreadSafety();

//! === CreateFftPlan ===
/// @brief Create a complex-to-complex 1d FFT plan
/// @param in   input data stream
/// @param out  fft result data stream
/// @param len  length of fft
/// @param make_fft boolean to decide whether to create FFT or IFFT plan
/// @return fftw_plan for FFT
fftw_plan Create1dFftPlan(
    Eigen::VectorXcd &in, Eigen::VectorXcd &out, const int len, bool make_fft = true);

//! === Create2dFftPlan ===
/// @brief Create multiple complex-to-complex 1d FFT plans
/// @param in   input data stream (nrow X ncol)
/// @param out  fft result data stream (nrow X ncol)
/// @param nrow number of ffts to perform
/// @param ncol length of fft
/// @param make_fft boolean to decide whether to create FFT or IFFT plan
/// @return fftw_plan for FFT
fftw_plan Create2dFftPlan(
    Eigen::MatrixXcd &in,
    Eigen::MatrixXcd &out,
    const int nrow,
    const int ncol,
    bool make_fft = true);

//! === ExecuteFftPlan ===
/// @brief Perform a complex-to-complex 1d FFT/IFFT
/// @param p    Generated fftw_plan
/// @return True|False based on success
bool ExecuteFftPlan(fftw_plan &p);

}  // end namespace sturdr

#endif