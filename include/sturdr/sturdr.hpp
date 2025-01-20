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
#include "sturdr/navigator.hpp"
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
  std::function<void(RfDataType[], std::complex<double>[], const int &)> DataTypeAdapterFunc_;
  uint64_t shm_ptr_;
  uint64_t shm_chunk_size_samp_;
  uint64_t shm_write_size_samp_;
  std::shared_ptr<Eigen::VectorXcd> shm_;
  std::shared_ptr<ConcurrentBarrier> start_barrier_;
  std::shared_ptr<ConcurrentBarrier> end_barrier_;
  std::vector<GpsL1caChannel> gps_channels_;
  std::shared_ptr<bool> running_;

  /**
   * @brief navigation parameters
   */
  std::shared_ptr<ConcurrentQueue<ChannelNavPacket>> nav_queue_;
  std::shared_ptr<ConcurrentQueue<ChannelEphemPacket>> eph_queue_;
  std::unique_ptr<Navigator> nav_;

  /**
   * @brief acquisition parameters
   */
  std::array<std::array<bool, 1023>, 32> codes_;
  std::vector<uint8_t> prn_available_;
  std::vector<uint8_t> prn_used_;
  int prn_ptr_;
  std::mutex prn_mtx_;
  AcquisitionSetup acq_setup_;

  /**
   * *=== Init ===*
   * @brief Initialize SDR
   */
  void Init(const std::string &yaml_fname);

  /**
   * @brief Thread safe function for a channel to request a new PRN
   * @param prn current channel prn
   */
  void GetNewPrn(uint8_t &prn, std::array<bool, 1023> &code);

  /**
   * *=== NavigationThread ===*
   * @brief Runs the thread controlling navigation inputs and outputs
   */
  void NavigationThread();

  /**
   * *=== ChannelNavPacketListener ===*
   * @brief Listens to the channels for navigation packet outputs
   */
  void ChannelNavPacketListener();

  /**
   * *=== ChannelEphemPacketListener ===*
   * @brief Listens to the channels for ephemeris packet outputs
   */
  void ChannelEphemPacketListener();
};

}  // namespace sturdr

#endif