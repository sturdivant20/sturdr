/**
fftw-wrapper.cpp

========= ==========================================================================================
  @file   sturdr/fftw-wrapper.cpp
  @brief  Small functions for using complex-1d FFTW3 methods specific to STURDR.
  @date   January 2025
  @author Daniel Sturdivant <sturdivant20@gmail.com>
========= ==========================================================================================
*/

#include "sturdr/fftw-wrapper.hpp"

#include <spdlog/spdlog.h>

#include <cstdlib>

namespace sturdr {

// *=== ~FftwWrapper ===*
FftwWrapper::~FftwWrapper() {
  fftw_destroy_plan(fft_);
  fftw_destroy_plan(ifft_);
  fftw_destroy_plan(fft_many_);
  fftw_destroy_plan(ifft_many_);
}

// *=== ThreadSafety ===*
void FftwWrapper::ThreadSafety() {
  fftw_make_planner_thread_safe();
}

// *=== Create1dFftPlan ===*
void FftwWrapper::Create1dFftPlan(const int len, const bool is_fft) {
  Eigen::VectorXd tmp(len);
  try {
    if (is_fft) {
      // fft
      fft_ = fftw_plan_dft_1d(
          len,
          reinterpret_cast<fftw_complex *>(tmp.data()),
          reinterpret_cast<fftw_complex *>(tmp.data()),
          FFTW_FORWARD,
          FFTW_ESTIMATE);  // FFTW_MEASURE
    } else {
      // ifft
      ifft_ = fftw_plan_dft_1d(
          len,
          reinterpret_cast<fftw_complex *>(tmp.data()),
          reinterpret_cast<fftw_complex *>(tmp.data()),
          FFTW_BACKWARD,
          FFTW_ESTIMATE);
    }
  } catch (std::exception &e) {
    spdlog::get("sturdr-console")
        ->error("fftw-wrapper.cpp Create1dFftPlan failed! Error -> {}", e.what());
    exit(EXIT_FAILURE);
  }
}

// *=== CreateManyFftPlan ===*
void FftwWrapper::CreateManyFftPlan(
    const int nrow, const int ncol, const bool is_fft, const bool is_rowwise) {
  try {
    Eigen::MatrixXd tmp(nrow, ncol);

    int rank = 1;  // rank/dimension of fft
    int n[1];      // how long each fft is
    int howmany, idist, odist, istride, ostride;
    if (is_rowwise) {
      n[0] = ncol;
      howmany = nrow;                  // how many transforms to compute
      idist = 1, odist = 1;            // ptr to sequential fft starting locations
      istride = nrow, ostride = nrow;  // how far apart each sample is
    } else {
      n[0] = nrow;
      howmany = ncol;
      idist = nrow, odist = nrow;
      istride = 1, ostride = 1;
    }
    int *inembed = n, *onembed = n;

    if (is_fft) {
      // fft
      fft_many_ = fftw_plan_many_dft(
          rank,
          n,
          howmany,
          reinterpret_cast<fftw_complex *>(tmp.data()),
          inembed,
          istride,
          idist,
          reinterpret_cast<fftw_complex *>(tmp.data()),
          onembed,
          ostride,
          odist,
          FFTW_FORWARD,
          FFTW_ESTIMATE);
    } else {
      // ifft
      ifft_many_ = fftw_plan_many_dft(
          rank,
          n,
          howmany,
          reinterpret_cast<fftw_complex *>(tmp.data()),
          inembed,
          istride,
          idist,
          reinterpret_cast<fftw_complex *>(tmp.data()),
          onembed,
          ostride,
          odist,
          FFTW_BACKWARD,
          FFTW_ESTIMATE);
    }
  } catch (std::exception &e) {
    spdlog::get("sturdr-console")
        ->error("fftw-wrapper.cpp CreateManyFftPlan failed! Error -> {}", e.what());
    exit(EXIT_FAILURE);
  }
}

// *=== ExecuteFftPlan ===*
bool FftwWrapper::ExecuteFftPlan(
    Eigen::Ref<Eigen::MatrixXcd> in,
    Eigen::Ref<Eigen::MatrixXcd> out,
    const bool is_fft,
    const bool is_many_fft) {
  try {
    // execute fft
    if (is_fft) {
      if (is_many_fft) {
        fftw_execute_dft(
            fft_many_,
            reinterpret_cast<fftw_complex *>(in.data()),
            reinterpret_cast<fftw_complex *>(out.data()));
      } else {
        fftw_execute_dft(
            fft_,
            reinterpret_cast<fftw_complex *>(in.data()),
            reinterpret_cast<fftw_complex *>(out.data()));
      }
    } else {
      if (is_many_fft) {
        fftw_execute_dft(
            ifft_many_,
            reinterpret_cast<fftw_complex *>(in.data()),
            reinterpret_cast<fftw_complex *>(out.data()));
      } else {
        fftw_execute_dft(
            ifft_,
            reinterpret_cast<fftw_complex *>(in.data()),
            reinterpret_cast<fftw_complex *>(out.data()));
      }
    }
    return true;
  } catch (std::exception const &e) {
    spdlog::get("sturdr-console")->error("fftw-wrapper.cpp ExecutePlan failed. ERROR {}", e.what());
    // return false;
    exit(EXIT_FAILURE);
  }
}
// bool ExecuteManyFftPlan(
//     const fftw_plan &p, Eigen::Ref<Eigen::MatrixXcd> in, Eigen::Ref<Eigen::MatrixXcd> out) {
//   try {
//     // execute fft
//     fftw_execute_dft(
//         p,
//         reinterpret_cast<fftw_complex *>(in.data()),
//         reinterpret_cast<fftw_complex *>(out.data()));
//     return true;
//   } catch (std::exception const &e) {
//     spdlog::get("sturdr-console")->error("fftw-wrapper.cpp ExecutePlan failed. ERROR {}",
//     e.what());
//     // return false;
//     exit(EXIT_FAILURE);
//   }
// };

}  // end namespace sturdr