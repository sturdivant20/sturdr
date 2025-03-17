/**
 * *fftw-wrapper.hpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/fftw-wrapper.hpp
 * @brief   Small functions for using complex-1d FFTW3 methods specific to STURDR.
 * @date    January 2025
 * @author  Daniel Sturdivant <sturdivant20@gmail.com>
 * =======  ========================================================================================
 */

#ifndef STURDR_FFTW_WRAPPER_HPP
#define STURDR_FFTW_WRAPPER_HPP

#include <fftw3.h>

#include <Eigen/Dense>

namespace sturdr {

class FftwWrapper {
 public:
  fftw_plan fft_;
  fftw_plan ifft_;
  fftw_plan fft_many_;
  fftw_plan ifft_many_;

  FftwWrapper() = default;
  ~FftwWrapper();

  //! === ThreadSafety ===
  /// @brief Ensures the creation of FFT planners does not conflict with other processes
  void ThreadSafety();

  //! === CreateFftPlan ===
  /// @brief Create a complex-to-complex 1d FFT plan
  /// @param in   input data stream
  /// @param out  fft result data stream
  /// @param len  length of fft
  /// @param is_fft boolean to decide whether to create FFT or IFFT plan
  /// @return fftw_plan for FFT
  void Create1dFftPlan(const int len, const bool is_fft = true);

  //! === CreateManyFftPlan ===
  /// @brief Create multiple complex-to-complex 1d FFT plans
  /// @param in   input data stream (nrow X ncol)
  /// @param out  fft result data stream (nrow X ncol)
  /// @param nrow number of ffts to perform
  /// @param ncol length of fft
  /// @param is_fft boolean to decide whether to create FFT or IFFT plan
  /// @return fftw_plan for FFT
  void CreateManyFftPlan(
      const int nrow, const int ncol, const bool is_fft = true, const bool is_rowwise = true);

  //! === ExecuteFftPlan ===
  /// @brief Perform a complex-to-complex 1d FFT/IFFT
  /// @param p    Generated fftw_plan
  /// @return True|False based on success
  bool ExecuteFftPlan(
      Eigen::Ref<Eigen::MatrixXcd> in,
      Eigen::Ref<Eigen::MatrixXcd> out,
      const bool is_fft = true,
      const bool is_many_fft = false);
  // bool ExecuteFftPlan(
  //     const fftw_plan &p, Eigen::Ref<Eigen::VectorXcd> in, Eigen::Ref<Eigen::VectorXcd> out);
  // bool ExecuteManyFftPlan(
  //     const fftw_plan &p, Eigen::Ref<Eigen::MatrixXcd> in, Eigen::Ref<Eigen::MatrixXcd> out);
};

}  // end namespace sturdr

#endif