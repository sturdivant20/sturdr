/**
 * *sturdr.cpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/sturdr.cpp
 * @brief   SturDR receiver implementation.
 * @date    January 2025
 * =======  ========================================================================================
 */

#include "sturdr/sturdr.hpp"

#include <fftw3.h>
#include <spdlog/async.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/stopwatch.h>

#include <chrono>
#include <complex>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "sturdr/data-type-adapters.hpp"
#include "sturdr/fftw-wrapper.hpp"

namespace sturdr {

SturDR::SturDR(const std::string yaml_fname)
    : yp_{sturdio::YamlParser(yaml_fname)},
      conf_{
          {yp_.GetVar<std::string>("scenario"),
           yp_.GetVar<std::string>("in_file"),
           yp_.GetVar<std::string>("out_folder"),
           spdlog::level::from_str(yp_.GetVar<std::string>("log_level")),
           yp_.GetVar<uint64_t>("ms_to_skip"),
           yp_.GetVar<uint64_t>("ms_to_process"),
           yp_.GetVar<uint16_t>("ms_chunk_size"),
           yp_.GetVar<uint16_t>("ms_read_size"),
           yp_.GetVar<double>("reference_pos_x"),
           yp_.GetVar<double>("reference_pos_y"),
           yp_.GetVar<double>("reference_pos_z")},
          {yp_.GetVar<double>("samp_freq"),
           yp_.GetVar<double>("intmd_freq"),
           yp_.GetVar<bool>("is_complex"),
           static_cast<uint8_t>(yp_.GetVar<uint16_t>("bit_depth")),
           yp_.GetVar<std::string>("signals"),
           static_cast<uint8_t>(yp_.GetVar<uint16_t>("max_channels"))},
          {yp_.GetVar<double>("threshold"),
           yp_.GetVar<double>("doppler_range"),
           yp_.GetVar<double>("doppler_step"),
           static_cast<uint8_t>(yp_.GetVar<uint16_t>("num_coh_per")),
           static_cast<uint8_t>(yp_.GetVar<uint16_t>("num_noncoh_per")),
           yp_.GetVar<uint16_t>("max_failed_attempts")},
          {yp_.GetVar<uint16_t>("min_converg_time_ms"),
           yp_.GetVar<double>("tap_epl_wide"),
           yp_.GetVar<double>("tap_epl"),
           yp_.GetVar<double>("tap_epl_narrow"),
           yp_.GetVar<double>("dll_bandwidth_wide"),
           yp_.GetVar<double>("pll_bandwidth_wide"),
           yp_.GetVar<double>("fll_bandwidth_wide"),
           yp_.GetVar<double>("dll_bandwidth"),
           yp_.GetVar<double>("pll_bandwidth"),
           yp_.GetVar<double>("fll_bandwidth"),
           yp_.GetVar<double>("dll_bandwidth_narrow"),
           yp_.GetVar<double>("pll_bandwidth_narrow"),
           yp_.GetVar<double>("fll_bandwidth_narrow"),
           yp_.GetVar<double>("cno_alpha")},
          {yp_.GetVar<bool>("use_psr"),
           yp_.GetVar<bool>("use_doppler"),
           yp_.GetVar<bool>("use_adr"),
           yp_.GetVar<bool>("use_cno"),
           yp_.GetVar<bool>("do_vt"),
           static_cast<uint8_t>(yp_.GetVar<uint16_t>("meas_freq")),
           yp_.GetVar<std::string>("clock_model"),
           yp_.GetVar<double>("process_std_vel"),
           yp_.GetVar<double>("process_std_att"),
           yp_.GetVar<double>("nominal_transit_time")},
          {yp_.GetVar<bool>("is_multi_antenna"),
           static_cast<uint8_t>(yp_.GetVar<uint16_t>("n_ant")),
           Eigen::MatrixXd::Zero(3, yp_.GetVar<int>("n_ant"))}},
      bf_(conf_.antenna.n_ant),
      samp_per_ms_{static_cast<uint64_t>(conf_.rfsignal.samp_freq) / 1000},
      shm_ptr_{0},
      shm_file_size_samp_{conf_.general.ms_chunk_size * samp_per_ms_},
      shm_read_size_samp_{conf_.general.ms_read_size * samp_per_ms_},
      shm_{std::make_shared<Eigen::MatrixXcd>(
          Eigen::MatrixXcd::Zero(shm_file_size_samp_, conf_.antenna.n_ant))},
      running_{std::make_shared<bool>(true)},
      n_dopp_bins_{
          2 * static_cast<uint64_t>(
                  conf_.acquisition.doppler_range / conf_.acquisition.doppler_step) +
          1},
      fftw_plans_{std::make_shared<FftwWrapper>()},
      prn_ptr_{1},
      barrier_{std::make_shared<ConcurrentBarrier>(conf_.rfsignal.max_channels + 1)},
      log_{spdlog::stdout_color_mt<spdlog::async_factory>("sturdr-console")},
      nav_queue_{std::make_shared<ConcurrentQueue<ChannelNavPacket>>()},
      eph_queue_{std::make_shared<ConcurrentQueue<ChannelEphemPacket>>()} {
  // setup terminal/console logger
  log_->set_pattern("\033[1;34m[%D %T.%e][%^%l%$\033[1;34m]: \033[0m%v");
  log_->set_level(conf_.general.log_level);
  log_->trace("scenario: {}", conf_.general.scenario);
  log_->trace("ms_to_process: {}", conf_.general.ms_to_process);
  log_->trace("ms_chunk_size: {}", conf_.general.ms_chunk_size);
  log_->trace("ms_read_size: {}", conf_.general.ms_read_size);
  log_->trace("in_file: {}", conf_.general.in_file);
  log_->trace("out_folder: {}", conf_.general.out_folder);
  log_->trace("reference_pos_x: {}", conf_.general.reference_pos_x);
  log_->trace("reference_pos_y: {}", conf_.general.reference_pos_y);
  log_->trace("reference_pos_z: {}", conf_.general.reference_pos_z);
  log_->trace("log_level: {}", spdlog::level::to_string_view(log_->level()));
  log_->trace("samp_freq: {}", conf_.rfsignal.samp_freq);
  log_->trace("intmd_freq: {}", conf_.rfsignal.intmd_freq);
  log_->trace("is_complex: {}", conf_.rfsignal.is_complex);
  log_->trace("bit_depth: {}", conf_.rfsignal.bit_depth);
  log_->trace("signals: {}", conf_.rfsignal.signals);
  log_->trace("max_channels: {}", conf_.rfsignal.max_channels);
  log_->trace("doppler_range: {}", conf_.acquisition.doppler_range);
  log_->trace("doppler_step: {}", conf_.acquisition.doppler_step);
  log_->trace("num_coh_per: {}", conf_.acquisition.num_coh_per);
  log_->trace("num_noncoh_per: {}", conf_.acquisition.num_noncoh_per);
  log_->trace("threshold: {}", conf_.acquisition.threshold);
  log_->trace("min_converg_time_ms: {}", conf_.tracking.min_converg_time_ms);
  log_->trace("tap_epl_wide: {}", conf_.tracking.tap_epl_wide);
  log_->trace("tap_epl: {}", conf_.tracking.tap_epl_standard);
  log_->trace("tap_epl_narrow: {}", conf_.tracking.tap_epl_narrow);
  log_->trace("pll_bandwidth_wide: {}", conf_.tracking.pll_bw_wide);
  log_->trace("fll_bandwidth_wide: {}", conf_.tracking.fll_bw_wide);
  log_->trace("dll_bandwidth_wide: {}", conf_.tracking.dll_bw_wide);
  log_->trace("pll_bandwidth: {}", conf_.tracking.pll_bw_standard);
  log_->trace("fll_bandwidth: {}", conf_.tracking.fll_bw_standard);
  log_->trace("dll_bandwidth: {}", conf_.tracking.dll_bw_standard);
  log_->trace("pll_bandwidth_narrow: {}", conf_.tracking.pll_bw_narrow);
  log_->trace("fll_bandwidth_narrow: {}", conf_.tracking.fll_bw_narrow);
  log_->trace("dll_bandwidth_narrow: {}", conf_.tracking.dll_bw_narrow);
  log_->trace("cno_alpha: {}", conf_.tracking.cno_alpha);
  log_->trace("meas_freq: {}", conf_.navigation.meas_freq);
  log_->trace("process_std_vel: {}", conf_.navigation.process_std_vel);
  log_->trace("process_std_att: {}", conf_.navigation.process_std_att);
  log_->trace("clock_model: {}", conf_.navigation.clock_model);
  log_->trace("nominal_transit_time: {}", conf_.navigation.nominal_transit_time);
  log_->trace("use_psr: {}", conf_.navigation.use_psr);
  log_->trace("use_doppler: {}", conf_.navigation.use_doppler);
  log_->trace("use_adr: {}", conf_.navigation.use_adr);
  log_->trace("use_cno: {}", conf_.navigation.use_cno);
  log_->trace("do_vt: {}", conf_.navigation.do_vt);

  // Create FFT plans
  fftw_plans_->Create1dFftPlan(samp_per_ms_, true);
  fftw_plans_->Create1dFftPlan(samp_per_ms_, false);
  fftw_plans_->CreateManyFftPlan(samp_per_ms_, n_dopp_bins_, true, false);
  fftw_plans_->CreateManyFftPlan(samp_per_ms_, n_dopp_bins_, false, false);

  // read in the antenna positions if necessary
  if (conf_.antenna.n_ant > 1) {
    std::vector<double> vec;
    std::string item;
    for (int i = 0; i < conf_.antenna.n_ant; i++) {
      item = "ant_xyz_" + std::to_string(i);
      yp_.GetVar<std::vector<double>>(vec, item);
      conf_.antenna.ant_xyz.col(i) = Eigen::Map<Eigen::VectorXd>(vec.data(), vec.size());
      vec.clear();
    }
    std::cout << "ant_pos: \n" << conf_.antenna.ant_xyz << "\n";
  }

  // start navigator
  navigator_ = std::make_unique<Navigator>(conf_, nav_queue_, eph_queue_, running_);
}

// *=== ~SturDR ===*
SturDR::~SturDR() {
  log_->trace("~SturDR");
  log_->info("SturDR shutting down ...");
}

// *=== Start ===*
void SturDR::Start() {
  // Initialize channels
  InitChannels();

  // choose correct data type adapter
  if (!conf_.antenna.is_multi_antenna) {
    // single antenna receiver
    bf_[0].fopen(conf_.general.in_file);

    if (!conf_.rfsignal.is_complex) {
      if (conf_.rfsignal.bit_depth == 8) {
        // byte
        Run<int8_t>();
      } else if (conf_.rfsignal.bit_depth == 16) {
        // short
        Run<int16_t>();
      } else if (conf_.rfsignal.bit_depth == 32) {
        // float
        Run<float>();
      }
    } else {
      if (conf_.rfsignal.bit_depth == 8) {
        // complex byte
        RunComplex<int8_t>();
      } else if (conf_.rfsignal.bit_depth == 16) {
        // complex short
        RunComplex<int16_t>();
      } else if (conf_.rfsignal.bit_depth == 32) {
        // complex float
        RunComplex<float>();
      }
    }
  } else {
    // multi antenna receiver
    std::string fname;
    for (int i = 0; i < conf_.antenna.n_ant; i++) {
      fname = conf_.general.in_file + "-" + std::to_string(i) + ".bin";
      log_->debug("Opening: {}", fname);
      bf_[i].fopen(fname);
    }

    if (!conf_.rfsignal.is_complex) {
      if (conf_.rfsignal.bit_depth == 8) {
        // byte
        RunArray<int8_t>();
      } else if (conf_.rfsignal.bit_depth == 16) {
        // short
        RunArray<int16_t>();
      } else if (conf_.rfsignal.bit_depth == 32) {
        // float
        RunArray<float>();
      }
    } else {
      if (conf_.rfsignal.bit_depth == 8) {
        // byte
        RunComplexArray<int8_t>();
      } else if (conf_.rfsignal.bit_depth == 16) {
        // short
        RunComplexArray<int16_t>();
      } else if (conf_.rfsignal.bit_depth == 32) {
        // float
        RunComplexArray<float>();
      }
    }
  }

  // end SturDR
  log_->info("SturDR killing threads ...");
  *running_ = false;
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  barrier_->NotifyComplete();
  nav_queue_->NotifyComplete();
  eph_queue_->NotifyComplete();
  for (uint8_t i = 0; i < (uint8_t)conf_.rfsignal.max_channels; i++) {
    if (!conf_.antenna.is_multi_antenna) {
      gps_l1ca_channels_[i].Join();
    } else {
      gps_l1ca_array_channels_[i].Join();
    }
  }
  navigator_->Notify();
}

// *=== GenNewPrn ===*
void SturDR::GetNewPrn(uint8_t& prn) {
  std::unique_lock<std::mutex> lock(prn_mtx_);

  // remove prn from log
  prns_in_use_[prn] = false;

  // search for next available prn
  while (prns_in_use_[prn_ptr_]) {
    prn_ptr_ = prn_ptr_ % 32 + 1;
  }

  // log new prn
  prn = prn_ptr_;
  prns_in_use_[prn_ptr_] = true;
  prn_ptr_ = prn_ptr_ % 32 + 1;
}

// *=== InitChannels ===*
void SturDR::InitChannels() {
  // TODO: more constellation initializers

  // initialize available prn map
  for (uint8_t i = 1; i <= 32; i++) {
    prns_in_use_.insert({i, false});
  }

  // initialize channels of requested type
  if (conf_.rfsignal.signals == "gps_l1ca") {
    std::function<void(uint8_t&)> get_new_prn_func =
        std::bind(&SturDR::GetNewPrn, this, std::placeholders::_1);

    if (!conf_.antenna.is_multi_antenna) {
      gps_l1ca_channels_.reserve(conf_.rfsignal.max_channels);
      for (uint8_t i = 1; i <= (uint8_t)conf_.rfsignal.max_channels; i++) {
        gps_l1ca_channels_.emplace_back(
            conf_,
            i,
            running_,
            shm_,
            barrier_,
            eph_queue_,
            nav_queue_,
            fftw_plans_,
            get_new_prn_func);
        gps_l1ca_channels_[i - 1].Start();
      }
    } else {
      gps_l1ca_array_channels_.reserve(conf_.rfsignal.max_channels);
      for (uint8_t i = 1; i <= (uint8_t)conf_.rfsignal.max_channels; i++) {
        gps_l1ca_array_channels_.emplace_back(
            conf_,
            i,
            running_,
            shm_,
            barrier_,
            eph_queue_,
            nav_queue_,
            fftw_plans_,
            get_new_prn_func);
        gps_l1ca_array_channels_[i - 1].Start();
      }
    }
  }
}

//! ------------------------------------------------------------------------------------------------

// *=== Run ===*
template <typename T>
void SturDR::Run() {
  spdlog::stopwatch sw;
  log_->info("Starting SturDR with real input");

  // initialize rf data stream
  std::vector<T> rf_stream(shm_read_size_samp_);
  bf_[0].fseek<T>(static_cast<int>(conf_.general.ms_to_skip * samp_per_ms_));
  bf_[0].fread<T>(rf_stream.data(), shm_read_size_samp_);
  TypeToIDouble<T>(
      rf_stream.data(),
      shm_->col(0).segment(shm_ptr_, shm_read_size_samp_).data(),
      shm_read_size_samp_);
  shm_ptr_ += shm_read_size_samp_;
  shm_ptr_ %= shm_file_size_samp_;

  // run channels for specified amount of time
  int meas_freq_ms = 1000 / (int)conf_.navigation.meas_freq;
  int read_freq_ms = (int)conf_.general.ms_read_size;
  int n = (int)conf_.general.ms_to_process + 1;
  int ndot = std::min(read_freq_ms, meas_freq_ms);

  for (int i = 0; i <= n; i += ndot) {
    // check for screen printouts every second
    if (!(i % 1000)) {
      log_->info("File time: {:.3f} s ... Processing Time: {:.3f} s", (float)i / 1000.0, sw);
    }

    // check if time for nav update
    if (!(i % meas_freq_ms)) {
      navigator_->Notify();
    }

    // check if time for new data to be parsed
    if (!(i % read_freq_ms)) {
      // ready to process data
      barrier_->Wait();

      // read next signal data while channels are processing
      bf_[0].fread<T>(rf_stream.data(), shm_read_size_samp_);
      TypeToIDouble<T>(
          rf_stream.data(),
          shm_->col(0).segment(shm_ptr_, shm_read_size_samp_).data(),
          shm_read_size_samp_);
      shm_ptr_ += shm_read_size_samp_;
      shm_ptr_ %= shm_file_size_samp_;
    }
  }
}

// *=== RunComplex ===*
template <typename T>
void SturDR::RunComplex() {
  spdlog::stopwatch sw;
  log_->info("Starting SturDR with complex input");

  // initialize rf data stream
  std::vector<std::complex<T>> rf_stream(shm_read_size_samp_);
  bf_[0].fseekc<T>(static_cast<int>(conf_.general.ms_to_skip * samp_per_ms_));
  bf_[0].freadc<T>(rf_stream.data(), shm_read_size_samp_);

  ITypeToIDouble<T>(
      rf_stream.data(),
      shm_->col(0).segment(shm_ptr_, shm_read_size_samp_).data(),
      shm_read_size_samp_);
  shm_ptr_ += shm_read_size_samp_;
  shm_ptr_ %= shm_file_size_samp_;

  // run channels for specified amount of time
  int meas_freq_ms = 1000 / (int)conf_.navigation.meas_freq;
  int read_freq_ms = (int)conf_.general.ms_read_size;
  int n = (int)conf_.general.ms_to_process + 1;
  int ndot = std::min(read_freq_ms, meas_freq_ms);

  for (int i = 0; i <= n; i += ndot) {
    // check for screen printouts every second
    if (!(i % 1000)) {
      log_->info("File time: {:.3f} s ... Processing Time: {:.3f} s", (float)i / 1000.0, sw);
    }

    // check if time for nav update
    if (!(i % meas_freq_ms)) {
      navigator_->Notify();
    }

    // check if time for new data to be parsed
    if (!(i % read_freq_ms)) {
      // ready to process data
      barrier_->Wait();

      // read next signal data while channels are processing
      bf_[0].freadc<T>(rf_stream.data(), shm_read_size_samp_);
      ITypeToIDouble<T>(
          rf_stream.data(),
          shm_->col(0).segment(shm_ptr_, shm_read_size_samp_).data(),
          shm_read_size_samp_);
      shm_ptr_ += shm_read_size_samp_;
      shm_ptr_ %= shm_file_size_samp_;
    }
  }
}

// *=== RunArray ===*
template <typename T>
void SturDR::RunArray() {
  spdlog::stopwatch sw;
  log_->info("Starting SturDR with real input and antenna array");

  // initialize rf data stream
  std::vector<T> rf_stream(shm_read_size_samp_);
  for (uint8_t j = 0; j < conf_.antenna.n_ant; j++) {
    bf_[j].fseek<T>(static_cast<int>(conf_.general.ms_to_skip * samp_per_ms_));
    bf_[j].fread<T>(rf_stream.data(), shm_read_size_samp_);
    TypeToIDouble<T>(
        rf_stream.data(),
        shm_->col(j).segment(shm_ptr_, shm_read_size_samp_).data(),
        shm_read_size_samp_);
  }
  shm_ptr_ += shm_read_size_samp_;
  shm_ptr_ %= shm_file_size_samp_;

  // run channels for specified amount of time
  int meas_freq_ms = 1000 / (int)conf_.navigation.meas_freq;
  int read_freq_ms = (int)conf_.general.ms_read_size;
  int n = (int)conf_.general.ms_to_process + 1;
  int ndot = std::min(read_freq_ms, meas_freq_ms);

  for (int i = 0; i <= n; i += ndot) {
    // check for screen printouts every second
    if (!(i % 1000)) {
      log_->info("File time: {:.3f} s ... Processing Time: {:.3f} s", (float)i / 1000.0, sw);
    }

    // check if time for nav update
    if (!(i % meas_freq_ms)) {
      navigator_->Notify();
    }

    // check if time for new data to be parsed
    if (!(i % read_freq_ms)) {
      // ready to process data
      barrier_->Wait();

      // read next signal data while channels are processing
      for (uint8_t j = 0; j < conf_.antenna.n_ant; j++) {
        bf_[j].fread<T>(rf_stream.data(), shm_read_size_samp_);
        TypeToIDouble<T>(
            rf_stream.data(),
            shm_->col(j).segment(shm_ptr_, shm_read_size_samp_).data(),
            shm_read_size_samp_);
      }
      shm_ptr_ += shm_read_size_samp_;
      shm_ptr_ %= shm_file_size_samp_;
    }
  }
}

// *=== RunComplexArray ===*
template <typename T>
void SturDR::RunComplexArray() {
  spdlog::stopwatch sw;
  log_->info("Starting SturDR with complex input and antenna array");

  // initialize rf data stream
  std::vector<std::complex<T>> rf_stream(shm_read_size_samp_);
  for (uint8_t j = 0; j < conf_.antenna.n_ant; j++) {
    bf_[j].fseekc<T>(static_cast<int>(conf_.general.ms_to_skip * samp_per_ms_));
    bf_[j].freadc<T>(rf_stream.data(), shm_read_size_samp_);
    ITypeToIDouble<T>(
        rf_stream.data(),
        shm_->col(j).segment(shm_ptr_, shm_read_size_samp_).data(),
        shm_read_size_samp_);
  }
  shm_ptr_ += shm_read_size_samp_;
  shm_ptr_ %= shm_file_size_samp_;

  // run channels for specified amount of time
  int meas_freq_ms = 1000 / (int)conf_.navigation.meas_freq;
  int read_freq_ms = (int)conf_.general.ms_read_size;
  int n = (int)conf_.general.ms_to_process + 1;
  int ndot = std::min(read_freq_ms, meas_freq_ms);

  for (int i = 0; i <= n; i += ndot) {
    // check for screen printouts every second
    if (!(i % 1000)) {
      log_->info("File time: {:.3f} s ... Processing Time: {:.3f} s", (float)i / 1000.0, sw);
    }

    // check if time for nav update
    if (!(i % meas_freq_ms)) {
      navigator_->Notify();
    }

    // check if time for new data to be parsed
    if (!(i % read_freq_ms)) {
      // ready to process data
      barrier_->Wait();

      // read next signal data while channels are processing
      for (uint8_t j = 0; j < conf_.antenna.n_ant; j++) {
        bf_[j].freadc<T>(rf_stream.data(), shm_read_size_samp_);
        ITypeToIDouble<T>(
            rf_stream.data(),
            shm_->col(j).segment(shm_ptr_, shm_read_size_samp_).data(),
            shm_read_size_samp_);
      }
      shm_ptr_ += shm_read_size_samp_;
      shm_ptr_ %= shm_file_size_samp_;
    }
  }
}

// Explicit instantiation of SturDR::Run
template void SturDR::Run<int8_t>();
template void SturDR::Run<int16_t>();
template void SturDR::Run<float>();
template void SturDR::RunArray<int8_t>();
template void SturDR::RunArray<int16_t>();
template void SturDR::RunArray<float>();
template void SturDR::RunComplex<int8_t>();
template void SturDR::RunComplex<int16_t>();
template void SturDR::RunComplex<float>();
template void SturDR::RunComplexArray<int8_t>();
template void SturDR::RunComplexArray<int16_t>();
template void SturDR::RunComplexArray<float>();

}  // namespace sturdr