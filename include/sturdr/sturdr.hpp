/**
 * *sturdr.hpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/sturdr.hpp
 * @brief   SturDR receiver implementation.
 * @date    January 2025
 * =======  ========================================================================================
 */

// TODO: add channel vectors for other GNSS signal types

#ifndef STURDR_STURDR_HPP
#define STURDR_STURDR_HPP

#include <spdlog/spdlog.h>

#include <Eigen/Dense>
#include <map>
#include <memory>
#include <sturdio/binary-file.hpp>
#include <sturdio/yaml-parser.hpp>
#include <vector>

#include "sturdr/channel-gps-l1ca-array.hpp"
#include "sturdr/channel-gps-l1ca.hpp"
#include "sturdr/concurrent-barrier.hpp"
#include "sturdr/concurrent-queue.hpp"
#include "sturdr/fftw-wrapper.hpp"
#include "sturdr/navigator.hpp"
#include "sturdr/structs-enums.hpp"

namespace sturdr {

class SturDR {
 private:
  /**
   * @brief configuration
   */
  sturdio::YamlParser yp_;
  Config conf_;
  std::vector<sturdio::BinaryFile> bf_;
  uint64_t samp_per_ms_;

  /**
   * @brief shared memory parameters
   */
  // std::function<void(T[], std::complex<double>[], const int &)> data_type_adapter_func_;
  uint64_t shm_ptr_;
  uint64_t shm_file_size_samp_;
  uint64_t shm_read_size_samp_;
  std::shared_ptr<Eigen::MatrixXcd> shm_;

  /**
   * @brief channel parameters
   * TODO: add more channel types, maybe abstractify?
   */
  std::shared_ptr<bool> running_;
  uint64_t n_dopp_bins_;
  std::shared_ptr<FftwWrapper> fftw_plans_;
  uint8_t prn_ptr_;
  std::map<uint8_t, bool> prns_in_use_;
  std::mutex prn_mtx_;
  std::vector<ChannelGpsL1ca> gps_l1ca_channels_;
  std::vector<ChannelGpsL1caArray> gps_l1ca_array_channels_;
  std::shared_ptr<ConcurrentBarrier> barrier_;

  /**
   * @brief spdlog loggers
   */
  std::shared_ptr<spdlog::logger> log_;

  /**
   * @brief navigation parameters
   */
  std::shared_ptr<ConcurrentQueue<ChannelNavPacket>> nav_queue_;
  std::shared_ptr<ConcurrentQueue<ChannelEphemPacket>> eph_queue_;
  std::unique_ptr<Navigator> navigator_;

 public:
  /**
   * *=== SturDR ===*
   * @brief constructor
   * @param yaml_fname string containing signal file name
   */
  SturDR(const std::string yaml_fname);

  /**
   * *=== ~SturDR ===*
   * @brief destructor
   */
  ~SturDR();

  /**
   * *=== Start ===*
   * @brief Initializes and begins running the receiver
   */
  void Start();

 private:
  /**
   * *=== Run ===*
   * @brief Runs the receiver using real input stream
   */
  template <typename T>
  void Run();
  template <typename T>
  void RunArray();

  /**
   * *=== Run ===*
   * @brief Runs the receiver using complex input stream
   */
  template <typename T>
  void RunComplex();
  template <typename T>
  void RunComplexArray();

  /**
   * @brief Thread safe function for a channel to request a new PRN
   * @param prn current channel prn
   */
  void GetNewPrn(uint8_t &prn);

  /**
   * *=== InitChannels ===*
   * @brief initializes channels to be used
   */
  void InitChannels();
};

}  // namespace sturdr

#endif