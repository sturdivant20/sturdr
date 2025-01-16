/**
 * *receiver.cpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/receiver.cpp
 * @brief   STURDR receiver implementation.
 * @date    December 2024
 * =======  ========================================================================================
 */

#include "sturdr/sturdr.hpp"

#include <spdlog/async.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/stopwatch.h>

#include <Eigen/Dense>
#include <functional>
#include <memory>
#include <satutils/code-gen.hpp>
#include <satutils/gnss-constants.hpp>
#include <sturdio/yaml-parser.hpp>

#include "sturdr/concurrent-barrier.hpp"
#include "sturdr/data-type-adapters.hpp"

namespace sturdr {

// *=== SturDR ===*
template <>
SturDR<int8_t>::SturDR(const std::string yaml_fname)  //
    : log_{spdlog::stdout_color_mt<spdlog::async_factory>("sturdr-console")},
      DataTypeAdapterFunc_{&ByteToIDouble},
      running_{std::make_shared<bool>(true)} {
  Init(yaml_fname);
}
template <>
SturDR<int16_t>::SturDR(const std::string yaml_fname)  //
    : log_{spdlog::stdout_color_mt<spdlog::async_factory>("sturdr-console")},
      DataTypeAdapterFunc_{&ShortToIDouble},
      running_{std::make_shared<bool>(true)} {
  Init(yaml_fname);
}
template <>
SturDR<std::complex<int8_t>>::SturDR(const std::string yaml_fname)
    : log_{spdlog::stdout_color_mt<spdlog::async_factory>("sturdr-console")},
      DataTypeAdapterFunc_{&IByteToIDouble},
      running_{std::make_shared<bool>(true)} {
  Init(yaml_fname);
}
template <>
SturDR<std::complex<int16_t>>::SturDR(const std::string yaml_fname)
    : log_{spdlog::stdout_color_mt<spdlog::async_factory>("sturdr-console")},
      DataTypeAdapterFunc_{&IShortToIDouble},
      running_{std::make_shared<bool>(true)} {
  Init(yaml_fname);
}

// *=== Run ===*
template <typename RfDataType>
void SturDR<RfDataType>::Run() {
  // initial read signal data
  fid_.fread(rf_stream_.data(), shm_write_size_samp_);
  DataTypeAdapterFunc_(
      rf_stream_.data(),
      shm_->segment(shm_ptr_, shm_write_size_samp_).data(),
      shm_write_size_samp_);
  shm_ptr_ += shm_write_size_samp_;
  shm_ptr_ %= shm_chunk_size_samp_;

  // run channels for specified amount of time
  spdlog::stopwatch sw;
  int n = (int)conf_.general.ms_to_process + 1;
  int ndot = (int)conf_.general.ms_read_size;
  for (int i = 0; i < n; i += ndot) {
    // ready to process data
    start_barrier_->Wait();

    // read next signal data while channels are processing
    fid_.fread(rf_stream_.data(), shm_write_size_samp_);
    DataTypeAdapterFunc_(
        rf_stream_.data(),
        shm_->segment(shm_ptr_, shm_write_size_samp_).data(),
        shm_write_size_samp_);
    shm_ptr_ += shm_write_size_samp_;
    shm_ptr_ %= shm_chunk_size_samp_;

    // check for screen printouts every second
    if (i % 1000 == 0) {
      log_->info("File time: {:.3f} s ... Processing Time: {:.3f} s", (float)i / 1000.0, sw);
    }

    // channels finished
    end_barrier_->Wait();
  }
  *running_ = false;

  // join channels
  for (int i = 0; i < (int)conf_.rfsignal.max_channels; i++) {
    gps_channels_[i].join();
  }
}

// *=== Init ===*
template <typename RfDataType>
void SturDR<RfDataType>::Init(const std::string &yaml_fname) {
  // parse yaml parameter file
  sturdio::YamlParser yp(yaml_fname);
  conf_.general.scenario = yp.GetVar<std::string>("scenario");
  conf_.general.in_file = yp.GetVar<std::string>("in_file");
  conf_.general.out_folder = yp.GetVar<std::string>("out_folder");
  conf_.general.log_level = spdlog::level::from_str(yp.GetVar<std::string>("log_level"));
  conf_.general.ms_to_process = yp.GetVar<uint64_t>("ms_to_process");
  conf_.general.ms_chunk_size = yp.GetVar<uint16_t>("ms_chunk_size");
  conf_.general.ms_read_size = yp.GetVar<uint16_t>("ms_read_size");
  conf_.general.reference_pos_x = yp.GetVar<double>("reference_pos_x");
  conf_.general.reference_pos_y = yp.GetVar<double>("reference_pos_y");
  conf_.general.reference_pos_z = yp.GetVar<double>("reference_pos_z");
  conf_.rfsignal.samp_freq = yp.GetVar<double>("samp_freq");
  conf_.rfsignal.intmd_freq = yp.GetVar<double>("intmd_freq");
  conf_.rfsignal.is_complex = yp.GetVar<bool>("is_complex");
  conf_.rfsignal.bit_depth = yp.GetVar<uint16_t>("bit_depth");
  conf_.rfsignal.signals = yp.GetVar<std::string>("signals");
  conf_.rfsignal.max_channels = yp.GetVar<uint16_t>("max_channels");
  conf_.acquisition.threshold = yp.GetVar<double>("threshold");
  conf_.acquisition.doppler_range = yp.GetVar<double>("doppler_range");
  conf_.acquisition.doppler_step = yp.GetVar<double>("doppler_step");
  conf_.acquisition.num_coh_per = yp.GetVar<uint16_t>("num_coh_per");
  conf_.acquisition.num_noncoh_per = yp.GetVar<uint16_t>("num_noncoh_per");
  conf_.acquisition.max_failed_attempts = yp.GetVar<uint16_t>("max_failed_attempts");
  conf_.tracking.min_converg_time_ms = yp.GetVar<uint16_t>("min_converg_time_ms");
  conf_.tracking.tap_epl_wide = yp.GetVar<double>("tap_epl_wide");
  conf_.tracking.tap_epl_standard = yp.GetVar<double>("tap_epl");
  conf_.tracking.tap_epl_narrow = yp.GetVar<double>("tap_epl_narrow");
  conf_.tracking.dll_bw_wide = yp.GetVar<double>("dll_bandwidth_wide");
  conf_.tracking.pll_bw_wide = yp.GetVar<double>("pll_bandwidth_wide");
  conf_.tracking.fll_bw_wide = yp.GetVar<double>("fll_bandwidth_wide");
  conf_.tracking.dll_bw_standard = yp.GetVar<double>("dll_bandwidth");
  conf_.tracking.pll_bw_standard = yp.GetVar<double>("pll_bandwidth");
  conf_.tracking.fll_bw_standard = yp.GetVar<double>("fll_bandwidth");
  conf_.tracking.dll_bw_narrow = yp.GetVar<double>("dll_bandwidth_narrow");
  conf_.tracking.pll_bw_narrow = yp.GetVar<double>("pll_bandwidth_narrow");
  conf_.tracking.fll_bw_narrow = yp.GetVar<double>("fll_bandwidth_narrow");
  conf_.navigation.use_psr = yp.GetVar<bool>("use_psr");
  conf_.navigation.use_doppler = yp.GetVar<bool>("use_doppler");
  conf_.navigation.use_adr = yp.GetVar<bool>("use_adr");
  conf_.navigation.use_cno = yp.GetVar<bool>("use_cno");
  conf_.navigation.do_vt = yp.GetVar<bool>("do_vt");
  conf_.navigation.meas_freq = yp.GetVar<uint16_t>("meas_freq");
  conf_.navigation.clock_model = yp.GetVar<std::string>("clock_model");
  conf_.navigation.process_std = yp.GetVar<double>("process_std");
  conf_.navigation.nominal_transit_time = yp.GetVar<double>("nominal_transit_time");

  // setup terminal/console logger
  log_->set_pattern("\033[1;34m[%D %T.%e][%^%l%$\033[1;34m]: \033[0m%v");
  log_->set_level(conf_.general.log_level);

  // setup shared memory access
  shm_ptr_ = 0;
  shm_chunk_size_samp_ = conf_.general.ms_chunk_size * conf_.rfsignal.samp_freq / 1000;
  shm_write_size_samp_ = conf_.general.ms_read_size * conf_.rfsignal.samp_freq / 1000;
  shm_ = std::make_shared<Eigen::VectorXcd>(Eigen::VectorXcd::Zero(shm_chunk_size_samp_));
  log_->debug(
      "SturDR shared memory vector initialized, Write Size: {}, Disk Size: {}",
      shm_write_size_samp_,
      shm_chunk_size_samp_);

  // setup binary file
  if (fid_.fopen(conf_.general.in_file)) {
    rf_stream_ = Eigen::Vector<RfDataType, Eigen::Dynamic>::Zero(shm_write_size_samp_);
    log_->debug("SturDR RF Data file opened: {}", conf_.general.in_file);
  }

  // setup channel queues
  nav_queue_ = std::make_shared<ConcurrentQueue<NavPacket>>();
  eph_queue_ = std::make_shared<ConcurrentQueue<EphemPacket>>();
  log_->debug("SturDR queues created!");

  // initialize thread syncronization barriers
  start_barrier_ = std::make_shared<ConcurrentBarrier>(conf_.rfsignal.max_channels + 1);
  end_barrier_ = std::make_shared<ConcurrentBarrier>(conf_.rfsignal.max_channels + 1);
  log_->debug("SturDR barriers created!");

  // initialize acquisition matrices
  ptr_prn_ = 0;
  for (int i = 0; i < 32; i++) {
    satutils::CodeGenCA(codes_[i], i + 1);
    available_prn_.push_back(i + 1);
  }
  acq_setup_ = sturdr::InitAcquisitionMatrices(
      codes_,
      conf_.acquisition.doppler_range,
      conf_.acquisition.doppler_step,
      conf_.rfsignal.samp_freq,
      satutils::GPS_CA_CODE_RATE<>,
      conf_.rfsignal.intmd_freq);
  log_->debug("SturDR initialized necessary acquisition matrices!");

  // initialize channels of requested type
  auto reg_func = std::bind(&SturDR<RfDataType>::GetNewPrn, this, std::placeholders::_1);
  // std::cout << "here 0\n";
  log_->debug("SturDR number of requested channels: {}", conf_.rfsignal.max_channels);
  gps_channels_.reserve(conf_.rfsignal.max_channels);
  for (int i = 0; i < (int)conf_.rfsignal.max_channels; i++) {
    used_prn_.push_back(i + 1);
    // std::cout << "here 1\n";
    gps_channels_.emplace_back(
        conf_,
        acq_setup_,
        shm_,
        nav_queue_,
        eph_queue_,
        start_barrier_,
        end_barrier_,
        i,
        running_,
        // std::bind(&SturDR<RfDataType>::GetNewPrn, this, std::placeholders::_1));
        // &SturDR<RfDataType>::GetNewPrn);
        reg_func);
    // std::cout << "here 2\n";
    gps_channels_[i].SetSatellite(i + 1);
    // std::cout << "here 3\n";
    gps_channels_[i].start();
    log_->info("Channel {} created and set to GPS{}!", i, i + 1);
  }
}

// *=== GetNewPrn ===*
template <typename RfDataType>
void SturDR<RfDataType>::GetNewPrn(uint8_t &prn) {
  std::unique_lock<std::mutex> lock(mtx_prn_);

  // remove prn from log
  if (used_prn_.size() > 0) {
    auto i = std::find(used_prn_.begin(), used_prn_.end(), prn);
    if (i != used_prn_.end()) {
      used_prn_.erase(i);
    }
  }

  // search for next available prn
  while (std::find(used_prn_.begin(), used_prn_.end(), available_prn_[ptr_prn_]) !=
         used_prn_.end()) {
    ptr_prn_ += 1;
    ptr_prn_ %= 32;
  }

  // log new prn
  prn = available_prn_[ptr_prn_];
  used_prn_.push_back(prn);
  ptr_prn_ += 1;
  ptr_prn_ %= 32;
}

// instantiate classes
template class SturDR<int8_t>;
template class SturDR<int16_t>;
template class SturDR<std::complex<int8_t>>;
template class SturDR<std::complex<int16_t>>;

}  // namespace sturdr