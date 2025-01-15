/**
 * *channel-gps-l1ca.hpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/channel-gps-l1ca.hpp
 * @brief   Implementation of channel for GPS L1 C/A signals.
 * @date    December 2024
 * @ref     1. "Understanding GPS/GNSS Principles and Applications", 3rd Edition, 2017
 *            - Kaplan & Hegarty
 *          2. "Global Positioning System: Signals, Measurements, and Performance", 2nd Edition,
 *              2006 - Misra & Enge
 *          3. "A Software-Defined GPS and Galileo Receiver: A Single-Frequency Approach", 2007
 *            - Borre, Akos, Bertelsen, Rinder, Jensen
 * =======  ========================================================================================
 */

#pragma once

#ifndef STURDR_CHANNEL_GPS_L1CA_HPP
#define STURDR_CHANNEL_GPS_L1CA_HPP

#include <spdlog/spdlog.h>

#include <array>
#include <complex>
#include <satutils/gps-lnav.hpp>

#include "sturdr/channel.hpp"
#include "sturdr/tracking.hpp"

namespace sturdr {

//! ------------------------------------------------------------------------------------------------

class GpsL1caChannel : public Channel {
 public:
  /**
   * *=== GpsL1caChannel ===*
   * @brief Constructor
   * @param config        configuration from yaml file
   * @param shm           shared memory array
   * @param channel_queue thread safe queue for sharing channel status messages
   * @param nav_queue     thread safe queue fir sharing navigation status messages
   * @param start_barrier thread safe barrier for synchronizing the start of channel processing
   * @param end_barrier   thread safe barrier for synchronizing the conclusion of channel processing
   * @param channel_num   STURDR channel creation/identification number
   */
  GpsL1caChannel(
      const Config &config,
      const SturdrFftPlans &fft_plans,
      std::shared_ptr<Eigen::VectorXcd> shm,
      std::shared_ptr<ConcurrentQueue<NavPacket>> nav_queue,
      std::shared_ptr<ConcurrentBarrier> start_barrier,
      std::shared_ptr<ConcurrentBarrier> end_barrier,
      int &channel_num,
      std::shared_ptr<bool> still_running);

  /**
   * *=== ~GpsL1caChannel ===*
   * @brief Destructor
   */
  ~GpsL1caChannel();

  /**
   * *=== SetSatellite ===*
   * @brief Set satellite tracked by channel to given prn
   */
  void SetSatellite(uint8_t sv_id);

 protected:
  // /**
  //  * *=== Run ===*
  //  * @brief Run channel processing
  //  */
  // void run();

  /**
   * *=== Acquire ===*
   * @brief Trys to acquire the set satellite
   */
  void Acquire();

  /**
   * *=== Track ===*
   * @brief Runs the tracking correlation and loop filter updates!
   */
  void Track();

  /**
   * *=== DataBitSync ===*
   * @brief Synchronize to the data bit and extend the integration periods
   */
  bool DataBitSync();

  /**
   * *=== Demodulate ===*
   * @brief Demodulate the navigation data and parse ephemerides
   */
  void Demodulate();

 private:
  // Code replica
  std::array<bool, 1023> code_;
  double rem_code_phase_;
  double code_doppler_;

  // Carrier replica
  double rem_carr_phase_;
  double carr_doppler_;
  double carr_jitter_;

  // NCO
  double T_;
  int PDI_;
  uint64_t total_samp_;
  uint64_t half_samp_;
  uint64_t samp_processed_;
  uint64_t samp_remaining_;
  uint64_t samp_per_ms_;
  uint64_t samp_per_chip_;

  // Tracking
  double kappa_;
  double w0d_;
  double w0p_;
  double w0f_;
  double tap_space_;
  std::complex<double> E_;
  std::complex<double> P_;
  std::complex<double> L_;
  std::complex<double> P1_;
  std::complex<double> P2_;
  std::complex<double> P_old_;
  TrackingKF filt_;
  uint8_t track_mode_;

  // Lock detectors
  double cno_mag_;
  double nbd_mem_;
  double nbp_mem_;
  double pc_mem_;
  double pn_mem_;
  bool code_lock_;
  bool carr_lock_;

  // Telemetry
  int cnt_;
  uint8_t bit_sync_hist_[20];
  satutils::GpsLnav<double> gps_lnav_;
};

}  // end namespace sturdr

#endif