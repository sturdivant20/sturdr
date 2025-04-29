/**
 * *channel-gps-l1ca.hpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/channel-gps-l1ca.hpp
 * @brief   Implementation of channel for GPS L1 C/A signals.
 * @date    January 2025
 * @ref     1. "Understanding GPS/GNSS Principles and Applications", 3rd Edition, 2017
 *            - Kaplan & Hegarty
 *          2. "Global Positioning System: Signals, Measurements, and Performance", 2nd Edition,
 *              2006 - Misra & Enge
 *          3. "A Software-Defined GPS and Galileo Receiver: A Single-Frequency Approach", 2007
 *            - Borre, Akos, Bertelsen, Rinder, Jensen
 * =======  ========================================================================================
 */

#ifndef STURDR_CHANNEL_GPS_L1CA_HPP
#define STURDR_CHANNEL_GPS_L1CA_HPP

#include <array>
#include <complex>
#include <satutils/gps-lnav.hpp>

#include "sturdr/channel.hpp"
#include "sturdr/fftw-wrapper.hpp"
#include "sturdr/lock-detectors.hpp"
#include "sturdr/tracking.hpp"

namespace sturdr {

class ChannelGpsL1ca : public Channel {
 protected:
  /**
   * @brief local replica properties and statistics
   */
  std::array<bool, 1023> code_;
  double intmd_freq_rad_;
  double rem_code_phase_;
  double code_doppler_;
  double rem_carr_phase_;
  double carr_doppler_;
  double carr_jitter_;
  double cno_;
  bool code_lock_;
  bool carr_lock_;
  LockDetectors lock_;
  // double nbd_;
  // double nbp_;
  // double pc_;
  // double pn_;
  double beta_sq_;
  double lambda_sq_;

  /**
   * @brief Tracking loops
   */
  uint8_t track_mode_;
  double kappa_;
  double w0d_;
  double w0p_;
  double w0f_;
  double tap_space_;
  Eigen::Vector<double, 12> noise_taps_;
  TrackingKF kf_;

  /**
   * @brief Correlators
   */
  std::complex<double> E_;
  std::complex<double> P_;
  std::complex<double> L_;
  std::complex<double> P1_;
  std::complex<double> P2_;
  std::complex<double> P_old_;
  Eigen::Vector<std::complex<double>, 12> N_;

  /**
   * @brief counters and telemetry
   */
  double T_;
  uint64_t T_ms_;
  uint64_t int_per_cnt_;
  uint64_t total_samp_;
  uint64_t half_samp_;
  uint64_t samp_remaining_;
  uint8_t bit_sync_hist_[20];
  satutils::GpsLnav<double> gps_lnav_;

 public:
  /**
   * *=== ChannelGpsL1ca ===*
   * @brief Implementation of channel for GPS L1CA signals
   */
  ChannelGpsL1ca(
      Config &conf,
      uint8_t &n,
      std::shared_ptr<bool> running,
      std::shared_ptr<Eigen::MatrixXcd> shared_array,
      std::shared_ptr<ConcurrentBarrier> barrier1,
      std::shared_ptr<ConcurrentBarrier> barrier2,
      std::shared_ptr<ConcurrentQueue> nav_queue,
      std::shared_ptr<FftwWrapper> fftw_plans,
      std::function<void(uint8_t &)> &GetNewPrnFunc);

  /**
   * *=== ~ChannelGpsL1ca ===*
   * @brief Destructor
   */
  ~ChannelGpsL1ca();

 protected:
  /**
   * *=== NewCodePeriod ===*
   * @brief begins a new code period
   */
  void NewCodePeriod();

  /**
   * *=== Acquire ===*
   * @brief Trys to acquire current satellite
   */
  void Acquire();

  /**
   * *=== Track ===*
   * @brief Trys to track current satellite
   */
  void Track();
  void Integrate(const uint64_t &samp_to_read);
  void Dump();
  void Status();

  /**
   * *=== NavDataSync ===*
   * @brief Trys to synchronize to the data bit and extend the integration periods
   */
  bool NavDataSync();

  /**
   * *=== Demodulate ===*
   * @brief Trys to demodulate navigation data and parse ephemerides
   */
  void Demodulate();
};

}  // namespace sturdr

#endif