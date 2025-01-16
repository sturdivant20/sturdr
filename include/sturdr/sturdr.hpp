/**
 * *sturdr.hpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/sturdr.hpp
 * @brief   STURDR receiver implementation.
 * @date    December 2024
 * =======  ========================================================================================
 */

// TODO: add channel vectors for other GNSS signal types

#ifndef STURDR_STURDR_HPP
#define STURDR_STURDR_HPP

#include <spdlog/spdlog.h>

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <sturdio/binary-file.hpp>
#include <vector>

#include "sturdr/acquisition.hpp"
#include "sturdr/channel-gps-l1ca.hpp"
#include "sturdr/concurrent-barrier.hpp"
#include "sturdr/concurrent-queue.hpp"
#include "sturdr/structs-enums.hpp"

namespace sturdr {

template <typename RfDataType>
class SturDR {
 public:
  /**
   * *=== SturDR ===*
   * @brief Constructor
   * @param yaml_fname string containing signal file name
   */
  SturDR(const std::string yaml_fname);

  /**
   * *=== Run ===*
   * @brief Run the receiver
   */
  void Run();

 private:
  /**
   * @brief configuration parameters
   */
  Config conf_;
  std::shared_ptr<spdlog::logger> log_;
  sturdio::BinaryFile fid_;
  Eigen::Vector<RfDataType, Eigen::Dynamic> rf_stream_;
  // void (*DataTypeAdapterFunc_)(const RfDataType[], std::complex<double>[], const int &);
  std::function<void(RfDataType[], std::complex<double>[], const int &)> DataTypeAdapterFunc_;
  uint64_t shm_ptr_;
  uint64_t shm_chunk_size_samp_;
  uint64_t shm_write_size_samp_;
  std::shared_ptr<Eigen::VectorXcd> shm_;
  std::shared_ptr<ConcurrentQueue<NavPacket>> nav_queue_;
  std::shared_ptr<ConcurrentQueue<EphemPacket>> eph_queue_;
  std::shared_ptr<ConcurrentBarrier> start_barrier_;
  std::shared_ptr<ConcurrentBarrier> end_barrier_;
  std::array<std::array<bool, 1023>, 32> codes_;
  std::vector<uint8_t> available_prn_;
  std::vector<uint8_t> used_prn_;
  int ptr_prn_;
  std::mutex mtx_prn_;
  AcquisitionSetup acq_setup_;
  std::shared_ptr<bool> running_;
  std::vector<GpsL1caChannel> gps_channels_;

  /**
   * *=== Init ===*
   * @brief Initialize SDR
   */
  void Init(const std::string &yaml_fname);

  /**
   * @brief Thread safe function for a channel to request a new PRN
   * @param prn current channel prn
   */
  void GetNewPrn(uint8_t &prn);
};

// template <typename RfDataType>
// class Receiver {
//  public:
//   /**
//    * *=== Receiver ===*
//    * @brief Constructor
//    * @param fname string containing signal file name (if path and fname_ is one string, this
//    * is the only input)
//    * @param fpath string containing path to signal file
//    */
//   Receiver(const std::string fname, const std::string fpath);
//   Receiver(const std::string fname);

//   /**
//    * *=== Receiver ===*
//    * @brief Destructor
//    */
//   ~Receiver();

//  private:
//   // Config
//   Config conf_;

//   // Channel items
//   SturdrFftPlans p;
//   std::shared_ptr<NavQueue> nav_queue_;
//   std::shared_ptr<EphQueue> eph_queue_;
//   std::shared_ptr<Barrier> start_barrier_;
//   std::shared_ptr<Barrier> end_barrier_;
//   std::shared_ptr<bool> still_running_;
//   std::vector<std::shared_ptr<Channel>> channels_;

//   // Signal data
//   sturdio::BinaryFile rf_file_;
//   DataTypeAdapter rf_adapter_;
//   std::vector<RfDataType> rf_data_;
//   Eigen::VectorXcd rf_shm_;
//   uint64_t shm_ptr_;
//   uint64_t shm_chunk_size_samp_;
//   uint64_t shm_write_size_samp_;

//   // navigation
//   std::shared_ptr<std::thread> nav_thread_;
//   // NavKF nav_kf_;

//   // spdlog
//   std::shared_ptr<spdlog::logger> log_;      // console/terminal logger
//   std::shared_ptr<spdlog::logger> nav_log_;  // navigation results logger
//   std::shared_ptr<spdlog::logger> eph_log_;  // ephemerides logger
// };

}  // end namespace sturdr

#endif