/**
 * *receiver.hpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/receiver.hpp
 * @brief   STURDR receiver implementation.
 * @date    December 2024
 * =======  ========================================================================================
 */

#pragma once

#ifndef STURDR_RECEIVER_HPP
#define STURDR_RECEIVER_HPP

#include <spdlog/async.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/stopwatch.h>

#include <Eigen/Dense>
#include <complex>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "sturdr/channel/channel.hpp"
#include "sturdr/channel/gps-l1ca-channel.hpp"
#include "sturdr/nav/ephemeris.hpp"
#include "sturdr/nav/estimation.hpp"
#include "sturdr/utils/concurrent-barrier.hpp"
#include "sturdr/utils/concurrent-queue.hpp"
#include "sturdr/utils/data-type-adapters.hpp"
#include "sturdr/utils/fftw-wrapper.hpp"
#include "sturdr/utils/io-tools.hpp"
#include "sturdr/utils/structs-enums.hpp"

using NavQueue = sturdr::ConcurrentQueue<sturdr::NavPacket>;
using EphQueue = sturdr::ConcurrentQueue<sturdr::EphemPacket>;
using Barrier = sturdr::ConcurrentBarrier;

namespace sturdr {

template <typename RfDataType>
class Receiver {
 public:
  /**
   * *=== Receiver ===*
   * @brief Constructor
   * @param fname string containing signal file name (if path and fname_ is one string, this
   * is the only input)
   * @param fpath string containing path to signal file
   */
  Receiver(const std::string fname, const std::string fpath);
  Receiver(const std::string fname);

  /**
   * *=== Receiver ===*
   * @brief Destructor
   */
  ~Receiver();

 private:
  // Config
  Config conf_;

  // Channel items
  SturdrFftPlans p;
  std::shared_ptr<NavQueue> nav_queue_;
  std::shared_ptr<EphQueue> eph_queue_;
  std::shared_ptr<Barrier> start_barrier_;
  std::shared_ptr<Barrier> end_barrier_;
  std::shared_ptr<bool> still_running_;
  std::vector<std::shared_ptr<Channel>> channels_;

  // Signal data
  RfDataFile rf_file_;
  DataTypeAdapter rf_adapter_;
  std::vector<RfDataType> rf_data_;
  Eigen::VectorXcd rf_shm_;
  uint64_t shm_ptr_;
  uint64_t shm_chunk_size_samp_;
  uint64_t shm_write_size_samp_;

  // navigation
  std::shared_ptr<std::thread> nav_thread_;
  NavKF nav_kf_;

  // spdlog
  std::shared_ptr<spdlog::logger> log_;      // console/terminal logger
  std::shared_ptr<spdlog::logger> nav_log_;  // navigation results logger
  std::shared_ptr<spdlog::logger> eph_log_;  // ephemerides logger
};

}  // end namespace sturdr

#endif