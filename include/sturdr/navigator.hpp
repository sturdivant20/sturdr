/**
 * *navigator.hpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/navigator.hpp
 * @brief   Handles navigation from the tracking states in each channel
 * @date    January 2025
 * @ref     1. "Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems", 2nd
 *              Edition, 2013 - Groves
 *          2. "Global Positioning System: Signals, Measurements, and Performance", 2nd Edition,
 *              2006 - Misra & Enge
 *          3. "A Software-Defined GPS and Galileo Receiver: A Single-Frequency Approach", 2007
 *            - Borre, Akos, Bertelsen, Rinder, Jensen
 * =======  ========================================================================================
 */

#ifndef STURDR_NAVIGATOR_HPP
#define STURDR_NAVIGATOR_HPP

#include <spdlog/spdlog.h>

#include <Eigen/Dense>
#include <any>
#include <fstream>
#include <map>
#include <memory>
#include <sturdins/kinematic-nav.hpp>
#include <thread>

#include "sturdr/concurrent-queue.hpp"
#include "sturdr/structs-enums.hpp"

namespace sturdr {

class Navigator {
 private:
  Config conf_;
  uint64_t file_size_;
  uint64_t nav_file_ptr_;
  bool is_init_;
  bool is_vector_;
  uint16_t n_ch_;

  std::thread thread_;
  std::shared_ptr<ConcurrentQueue> queue_;
  std::shared_ptr<bool> running_;
  std::any msg_;

  sturdins::KinematicNav kf_;
  uint16_t week_;
  double receive_time_;
  uint64_t ms_elapsed_;
  std::map<uint8_t, ChannelNavData> ch_data_;

  std::shared_ptr<spdlog::logger> log_;
  std::shared_ptr<std::ofstream> nav_log_;
  std::shared_ptr<std::ofstream> eph_log_;

 public:
  Navigator(Config& conf, std::shared_ptr<ConcurrentQueue> queue, std::shared_ptr<bool> running);
  ~Navigator();

  void NavThread();
  void NavUpdate();
  void ChannelUpdate(ChannelNavPacket& msg);
  void EphemUpdate(ChannelEphemPacket& msg);
  void ScalarUpdate();
  bool VectorUpdate();
  void LogNavData();

  /**
   * *=== UpdateShmWriterPtr ===*
   * @brief Keeps track of where the shm_ writer is so channel does not read more than is
   * written
   */
  void UpdateFilePtr(const uint64_t& d_samp);

  /**
   * *=== UnreadSampleCount ===*
   * @brief Returns the difference between shm_write_ptr_ and shm_
   */
  uint64_t GetDeltaSamples(const uint64_t& new_file_ptr);
};

}  // namespace sturdr

#endif