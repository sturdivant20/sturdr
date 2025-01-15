/**
fftw-wrapper.cpp

========= ==========================================================================================
  @file   sturdr/fftw-wrapper.cpp
  @brief  Small functions for using complex-1d FFTW3 methods specific to STURDR.
  @date   December 2024
  @author Daniel Sturdivant <sturdivant20@gmail.com>
========= ==========================================================================================
*/

#include "sturdr/fftw-wrapper.hpp"

#include <spdlog/spdlog.h>

#include <cstdlib>

namespace sturdr {

// *=== ThreadSafety ===*
void ThreadSafety() {
  fftw_make_planner_thread_safe();
};

// *=== Create1dFftPlan ===*
fftw_plan Create1dFftPlan(const int len, const bool make_fft) {
  Eigen::VectorXd tmp(len);
  try {
    if (make_fft) {
      // fft
      return fftw_plan_dft_1d(
          len,
          reinterpret_cast<fftw_complex *>(tmp.data()),
          reinterpret_cast<fftw_complex *>(tmp.data()),
          FFTW_FORWARD,
          FFTW_ESTIMATE);  // FFTW_MEASURE
    } else {
      // ifft
      return fftw_plan_dft_1d(
          len,
          reinterpret_cast<fftw_complex *>(tmp.data()),
          reinterpret_cast<fftw_complex *>(tmp.data()),
          FFTW_BACKWARD,
          FFTW_ESTIMATE);
    }
  } catch (std::exception &e) {
    spdlog::get("sturdr-console")
        ->error("fftw-wrapper.cpp CreatePlan failed! Error -> {}", e.what());
    exit(EXIT_FAILURE);
  }
};
// fftw_plan Create1dFftPlan(
//     Eigen::Ref<Eigen::VectorXcd> in,
//     Eigen::Ref<Eigen::VectorXcd> out,
//     const int len,
//     bool make_fft) {
//   fftw_plan p;
//   try {
//     if (make_fft) {
//       // fft
//       p = fftw_plan_dft_1d(
//           len,
//           reinterpret_cast<fftw_complex *>(in.data()),
//           reinterpret_cast<fftw_complex *>(out.data()),
//           FFTW_FORWARD,
//           FFTW_ESTIMATE);  // FFTW_MEASURE
//     } else {
//       // ifft
//       p = fftw_plan_dft_1d(
//           len,
//           reinterpret_cast<fftw_complex *>(in.data()),
//           reinterpret_cast<fftw_complex *>(out.data()),
//           FFTW_BACKWARD,
//           FFTW_ESTIMATE);
//     }
//   } catch (std::exception &e) {
//     spdlog::get("sturdr-console")
//         ->error("fftw-wrapper.cpp CreatePlan failed! Error -> {}", e.what());
//   }
//   return p;
// };

// *=== CreateManyFftPlan ===*
fftw_plan CreateManyFftPlan(const int nrow, const int ncol, const bool make_fft) {
  Eigen::MatrixXd tmp(nrow, ncol);

  int rank = 1;
  int n[] = {ncol};
  int howmany = nrow;
  int idist = 1, odist = 1;
  int istride = nrow, ostride = nrow;
  int *inembed = n, *onembed = n;

  try {
    if (make_fft) {
      // fft
      return fftw_plan_many_dft(
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
      return fftw_plan_many_dft(
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
        ->error("fftw-wrapper.cpp CreatePlan failed! Error -> {}", e.what());
    exit(EXIT_FAILURE);
  }
};
// fftw_plan CreateManyFftPlan(
//     Eigen::Ref<Eigen::MatrixXcd> in,
//     Eigen::Ref<Eigen::MatrixXcd> out,
//     const int nrow,
//     const int ncol,
//     bool make_fft) {
//   fftw_plan p;
//   int rank = 1;
//   int n[] = {ncol};
//   int howmany = nrow;
//   int idist = 1, odist = 1;
//   int istride = nrow, ostride = nrow;
//   int *inembed = n, *onembed = n;

//   try {
//     if (make_fft) {
//       // fft
//       p = fftw_plan_many_dft(
//           rank,
//           n,
//           howmany,
//           reinterpret_cast<fftw_complex *>(in.data()),
//           inembed,
//           istride,
//           idist,
//           reinterpret_cast<fftw_complex *>(out.data()),
//           onembed,
//           ostride,
//           odist,
//           FFTW_FORWARD,
//           FFTW_ESTIMATE);
//     } else {
//       // ifft
//       p = fftw_plan_many_dft(
//           rank,
//           n,
//           howmany,
//           reinterpret_cast<fftw_complex *>(in.data()),
//           inembed,
//           istride,
//           idist,
//           reinterpret_cast<fftw_complex *>(out.data()),
//           onembed,
//           ostride,
//           odist,
//           FFTW_BACKWARD,
//           FFTW_ESTIMATE);
//     }
//   } catch (std::exception &e) {
//     spdlog::get("sturdr-console")
//         ->error("fftw-wrapper.cpp CreatePlan failed! Error -> {}", e.what());
//   }
//   return p;
// };

// *=== ExecuteFftPlan ===*
bool ExecuteFftPlan(
    const fftw_plan &p, Eigen::Ref<Eigen::VectorXcd> in, Eigen::Ref<Eigen::VectorXcd> out) {
  try {
    // execute fft
    fftw_execute_dft(
        p,
        reinterpret_cast<fftw_complex *>(in.data()),
        reinterpret_cast<fftw_complex *>(out.data()));
    return true;
  } catch (std::exception const &e) {
    spdlog::get("sturdr-console")->error("fftw-wrapper.cpp ExecutePlan failed. ERROR {}", e.what());
    // return false;
    exit(EXIT_FAILURE);
  }
};
bool ExecuteManyFftPlan(
    const fftw_plan &p, Eigen::Ref<Eigen::MatrixXcd> in, Eigen::Ref<Eigen::MatrixXcd> out) {
  try {
    // execute fft
    fftw_execute_dft(
        p,
        reinterpret_cast<fftw_complex *>(in.data()),
        reinterpret_cast<fftw_complex *>(out.data()));
    return true;
  } catch (std::exception const &e) {
    spdlog::get("sturdr-console")->error("fftw-wrapper.cpp ExecutePlan failed. ERROR {}", e.what());
    // return false;
    exit(EXIT_FAILURE);
  }
};
// bool ExecuteFftPlan(fftw_plan &p) {
//   try {
//     // execute fft
//     fftw_execute(p);
//     return true;
//   } catch (std::exception const &e) {
//     spdlog::get("sturdr-console")->error("fftw-wrapper.cpp ExecutePlan failed. ERROR {}",
//     e.what()); return false;
//   }
// };

}  // end namespace sturdr