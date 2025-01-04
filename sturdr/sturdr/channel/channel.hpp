/**
 * *channel.hpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/channel/channel.hpp
 * @brief   Abstract class for GNSS channel definitions.
 * @date    December 2024
 * @ref     1. "Understanding GPS/GNSS Principles and Applications", 3rd Edition, 2017
 *             - Kaplan & Hegarty
 *          2. "A Software-Defined GPS and Galileo Receiver: A Single-Frequency Approach", 2007
 *             - Borre, Akos, Bertelsen, Rinder, Jensen
 * =======  ========================================================================================
 */

#ifndef STURDR_CHANNEL_HPP
#define STURDR_CHANNEL_HPP

#include <Eigen/Dense>
#include <chrono>
#include <complex>
#include <memory>
#include <navtools/constants.hpp>
#include <thread>

#include "sturdr/utils/concurrent-barrier.hpp"
#include "sturdr/utils/concurrent-queue.hpp"
#include "sturdr/utils/fftw-wrapper.hpp"
#include "sturdr/utils/structs-enums.hpp"

namespace sturdr {

class Channel {
 public:
  /**
   * *=== Channel ===*
   * @brief Constructor
   * @param config        configuration from yaml file
   * @param shm           shared memory array
   * @param channel_queue thread safe queue for sharing channel status messages
   * @param nav_queue     thread safe queue fir sharing navigation status messages
   * @param start_barrier thread safe barrier for synchronizing the start of channel processing
   * @param end_barrier   thread safe barrier for synchronizing the conclusion of channel processing
   * @param channel_num   STURDR channel creation/identification number
   */
  Channel(
      const Config &config,
      const SturdrFftPlans &fft_plans,
      std::shared_ptr<Eigen::VectorXcd> shm,
      std::shared_ptr<ConcurrentQueue<ChannelPacket>> channel_queue,
      std::shared_ptr<ConcurrentQueue<NavPacket>> nav_queue,
      std::shared_ptr<ConcurrentBarrier> start_barrier,
      std::shared_ptr<ConcurrentBarrier> end_barrier,
      int &channel_num,
      std::shared_ptr<bool> still_running)
      : conf_{config},
        shm_{shm},
        shm_read_ptr_{0},
        shm_write_ptr_{0},
        start_bar_{start_barrier},
        end_bar_{end_barrier},
        channel_queue_{channel_queue},
        nav_queue_{nav_queue},
        fft_plans_{fft_plans},
        intmd_freq_rad_{navtools::TWO_PI<double> * config.rfsignal.intmd_freq},
        still_running_{still_running},
        timeout_{100},
        log_{spdlog::get("sturdr-console")} {
    channel_id_ = "Sturdr_Ch" + std::to_string(channel_num);
    shm_samp_chunk_size_ = conf_.general.ms_chunk_size * conf_.rfsignal.samp_freq / 1000;
    shm_samp_write_size_ = conf_.general.ms_read_size * conf_.rfsignal.samp_freq / 1000;
  };

  /**
   * *=== ~Channel ===*
   * @brief Destructor
   */
  virtual ~Channel(){};

  /**
   * *=== start ===*
   * @brief Places all the channel operations inside a separate thread
   */
  // // virtual void start() = 0;
  void start() {
    log_->debug("Channel::start starting thread!");
    thread_ = std::make_shared<std::thread>(&Channel::run, this);
  };

  /**
   * *=== join ===*
   * @brief Kills the channel thread and joins back with the main process
   */
  // virtual void join() = 0;
  void join() {
    if (thread_->joinable()) {
      thread_->join();
    }
  };

  /**
   * @brief abstract functions to be defined per specific channel specifications
   */
  virtual void SetSatellite(uint8_t sv_id) = 0;

 protected:
  std::string channel_id_;
  Config conf_;
  ChannelPacket channel_msg_;
  NavPacket nav_msg_;
  std::shared_ptr<Eigen::VectorXcd> shm_;  // shared memory array
  uint64_t shm_read_ptr_;                  // location of channel inside shm_ array
  uint64_t shm_write_ptr_;                 // location of rfdata stream writer inside shm_ array
  uint64_t shm_samp_chunk_size_;           // size of shm in samples
  uint64_t shm_samp_write_size_;           // size of shm writer updates
  std::shared_ptr<ConcurrentBarrier> start_bar_;  // barrier synchronizing shm_ memory
  std::shared_ptr<ConcurrentBarrier> end_bar_;    // barrier synchronizing channel processing
  std::shared_ptr<ConcurrentQueue<ChannelPacket>> channel_queue_;  // queue for channel messages
  std::shared_ptr<ConcurrentQueue<NavPacket>> nav_queue_;          // queue for navigation messages
  std::shared_ptr<std::thread> thread_;
  SturdrFftPlans fft_plans_;  // bool to indicate processing is still being performed
  double intmd_freq_rad_;
  double nco_code_freq_;
  double nco_carr_freq_;
  std::shared_ptr<bool> still_running_;
  std::chrono::milliseconds timeout_;
  std::shared_ptr<spdlog::logger> log_;

  /**
   * *=== UpdateShmWriterPtr ===*
   * @brief Keeps track of where the shm_ writer is so channel does not pass the writer
   */
  void UpdateShmWriterPtr() {
    shm_write_ptr_ += shm_samp_write_size_;
    shm_write_ptr_ %= shm_samp_chunk_size_;
  }

  /**
   * *=== UnreadSampleCount ===*
   * @brief Returns the difference between shm_write_ptr_ and shm_
   */
  uint64_t UnreadSampleCount() {
    if (shm_read_ptr_ <= shm_write_ptr_) {
      return shm_write_ptr_ - shm_read_ptr_;
    } else {
      return shm_samp_chunk_size_ - shm_read_ptr_ + shm_write_ptr_;
    }
  }

  /**
   * @brief abstract functions to be defined per specific channel specifications
   */
  virtual void run() = 0;
  virtual void Acquire() = 0;
  virtual void Track() = 0;
  virtual bool DataBitSync() = 0;
  virtual void Demodulate() = 0;
};

// Channel::~Channel(){};

}  // end namespace sturdr

#endif