/**
fftw-wrapper.cpp

========= ==========================================================================================
  @file   sturdr/utils/fftw-wrapper.cpp
  @brief  Small functions for using complex-1d FFTW3 methods specific to STURDR.
  @date   December 2024
  @author Daniel Sturdivant <sturdivant20@gmail.com>
========= ==========================================================================================
*/

#include "sturdr/utils/fftw-wrapper.hpp"

#include <spdlog/spdlog.h>

namespace sturdr {

// *=== ThreadSafety ===*
void ThreadSafety() {
  fftw_make_planner_thread_safe();
};

// *=== Create1dFftPlan ===*
fftw_plan Create1dFftPlan(
    Eigen::VectorXcd &in, Eigen::VectorXcd &out, const int len, bool make_fft) {
  fftw_plan p;
  try {
    if (make_fft) {
      // fft
      p = fftw_plan_dft_1d(
          len,
          reinterpret_cast<fftw_complex *>(in.data()),
          reinterpret_cast<fftw_complex *>(out.data()),
          FFTW_FORWARD,
          FFTW_ESTIMATE);  // FFTW_MEASURE
    } else {
      // ifft
      p = fftw_plan_dft_1d(
          len,
          reinterpret_cast<fftw_complex *>(in.data()),
          reinterpret_cast<fftw_complex *>(out.data()),
          FFTW_BACKWARD,
          FFTW_ESTIMATE);
    }
  } catch (std::exception &e) {
    spdlog::default_logger()->error("fftw-wrapper.cpp CreatePlan failed! Error -> {}", e.what());
  }
  return p;
};

// *=== Create2dFftPlan ===*
fftw_plan Create2dFftPlan(
    Eigen::MatrixXcd &in, Eigen::MatrixXcd &out, const int nrow, const int ncol, bool make_fft) {
  fftw_plan p;
  int rank = 1;
  int n[] = {ncol};
  int howmany = nrow;
  int idist = 1, odist = 1;
  int istride = nrow, ostride = nrow;
  int *inembed = n, *onembed = n;

  try {
    if (make_fft) {
      // fft
      p = fftw_plan_many_dft(
          rank,
          n,
          howmany,
          reinterpret_cast<fftw_complex *>(in.data()),
          inembed,
          istride,
          idist,
          reinterpret_cast<fftw_complex *>(out.data()),
          onembed,
          ostride,
          odist,
          FFTW_FORWARD,
          FFTW_ESTIMATE);
    } else {
      // ifft
      p = fftw_plan_many_dft(
          rank,
          n,
          howmany,
          reinterpret_cast<fftw_complex *>(in.data()),
          inembed,
          istride,
          idist,
          reinterpret_cast<fftw_complex *>(out.data()),
          onembed,
          ostride,
          odist,
          FFTW_BACKWARD,
          FFTW_ESTIMATE);
    }
  } catch (std::exception &e) {
    spdlog::default_logger()->error("fftw-wrapper.cpp CreatePlan failed! Error -> {}", e.what());
  }
  return p;
};

// *=== ExecuteFftPlan ===*
bool ExecuteFftPlan(fftw_plan &p) {
  try {
    // execute fft
    fftw_execute(p);
    return true;
  } catch (std::exception const &e) {
    spdlog::default_logger()->error("fftw-wrapper.cpp ExecutePlan failed. ERROR {}", e.what());
    return false;
  }
};

}  // end namespace sturdr