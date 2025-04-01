/**
 * *channel-gps-l1ca-array.hpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/channel-gps-l1ca-array.hpp
 * @brief   Implementation of channel for antenna array using GPS L1 C/A signals.
 * @date    February 2025
 * @ref     1. "Deeply Integrated GNSS-INS with CRPA to Constrain Attitude Biases in Robust
 *              Navigators", GNSS+ 2024 - Daniel Sturdivant
 *          2. "A Multi-Antenna Vector Tracking Beamsteering GPS Receiver for Robust Positioning
 *              (Master's Thesis, Auburn University) - Scott Burchfield
 *          3. "Error Analysis of Carrier Phase Positioning Using Controlled Reception Pattern
 *              Antenna Arrays" (Master's thesis, Auburn University) - Josh Starling
 * =======  ========================================================================================
 */

#ifndef STURDR_CHANNEL_GPS_L1CA_ARRAY_HPP
#define STURDR_CHANNEL_GPS_L1CA_ARRAY_HPP

#include <Eigen/Dense>

#include "sturdr/beamformer.hpp"
#include "sturdr/channel-gps-l1ca.hpp"
#include "sturdr/fftw-wrapper.hpp"

namespace sturdr {

class ChannelGpsL1caArray : public ChannelGpsL1ca {
 protected:
  Eigen::Vector3d u_body_;
  Eigen::VectorXcd p_array_;
  Eigen::VectorXcd p1_array_;
  Eigen::VectorXcd p2_array_;
  BeamFormer bf_;

 public:
  /**
   * *=== ChannelGpsL1caArray ===*
   * @brief Implementation of channel for antenna array using GPS L1CA signals
   */
  ChannelGpsL1caArray(
      Config &conf,
      uint8_t &n,
      std::shared_ptr<bool> running,
      std::shared_ptr<Eigen::MatrixXcd> shared_array,
      std::shared_ptr<ConcurrentBarrier> start_barrier,
      std::shared_ptr<ConcurrentQueue> nav_queue,
      std::shared_ptr<FftwWrapper> fftw_plans,
      std::function<void(uint8_t &)> &GetNewPrnFunc);

  /**
   * *=== ~ChannelGpsL1caArray ===*
   * @brief Destructor
   */
  ~ChannelGpsL1caArray();

  /**
   * *=== Track ===*
   * @brief Trys to track current satellite using antenna array processing
   */
  // void Track() override;
  void Integrate(const uint64_t &samp_to_read) override;
  void Dump() override;

  /**
   * *=== NavDataSync ===*
   * @brief Trys to synchronize to the data bit and extend the integration periods
   */
  // bool NavDataSync() override;
};

}  // namespace sturdr

#endif