/**
 * *channel-gps-l1ca-array.cpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/channel-gps-l1ca.hpp
 * @brief   Implementation of channel for antenna array using GPS L1 C/A signals.
 * @date    February 2025
 * @ref     1. "Understanding GPS/GNSS Principles and Applications", 3rd Edition, 2017
 *            - Kaplan & Hegarty
 *          2. "Global Positioning System: Signals, Measurements, and Performance", 2nd Edition,
 *              2006 - Misra & Enge
 *          3. "A Software-Defined GPS and Galileo Receiver: A Single-Frequency Approach", 2007
 *            - Borre, Akos, Bertelsen, Rinder, Jensen
 * =======  ========================================================================================
 */

#include "sturdr/channel-gps-l1ca-array.hpp"

#include "sturdr/beamsteer.hpp"
#include "sturdr/gnss-signal.hpp"

namespace sturdr {

// *=== ChannelGpsL1caArray ===*
ChannelGpsL1caArray::ChannelGpsL1caArray(
    Config &conf,
    uint8_t &n,
    std::shared_ptr<bool> running,
    std::shared_ptr<Eigen::MatrixXcd> shared_array,
    std::shared_ptr<ConcurrentBarrier> start_barrier,
    std::shared_ptr<ConcurrentQueue<ChannelEphemPacket>> eph_queue,
    std::shared_ptr<ConcurrentQueue<ChannelNavPacket>> nav_queue,
    FftPlans &fftw_plans,
    std::function<void(uint8_t &)> &GetNewPrnFunc)
    : ChannelGpsL1ca(
          conf,
          n,
          running,
          shared_array,
          start_barrier,
          eph_queue,
          nav_queue,
          fftw_plans,
          GetNewPrnFunc),
      mu_{5e-9},
      W_{Eigen::VectorXcd::Zero(conf_.rfsignal.n_ant)},
      W_err_{Eigen::VectorXcd::Zero(conf_.rfsignal.n_ant)} {
  W_(0) = 1.0;
}

// *=== ~ChannelGpsL1caArray ===*
ChannelGpsL1caArray::~ChannelGpsL1caArray() {
}

// *=== Track ===*
void ChannelGpsL1caArray::Track() {
  // make sure a count of the unprocessed samples is made
  uint64_t unread_samples = UnreadSampleCount();
  uint64_t samp_to_read;

  // process until all samples have been read
  while (unread_samples > 0) {
    if (samp_remaining_ > 0) {
      // --- INTEGRATE ---
      samp_to_read = std::min(samp_remaining_, unread_samples);
      // std::cout << "samp_to_read: " << samp_to_read << ", samp_remaining: " << samp_remaining_
      //           << ", unread_samples: " << unread_samples << "\n";
      Integrate(samp_to_read);

    } else {
      // --- DUMP ---
      Dump();
      // W_ += W_err_;
      // LmsNormalize(W_);
    }

    // recount unread samples
    unread_samples = UnreadSampleCount();
  }

  // send navigation parameters precise to current sample
  if (!(*nav_pkt_.is_vector)) {
    nav_pkt_.FilePtr = shm_ptr_;
    nav_pkt_.CodePhase = rem_code_phase_;
    nav_pkt_.Doppler = carr_doppler_;
    nav_pkt_.CarrierPhase = rem_carr_phase_;
    q_nav_->push(nav_pkt_);
    {
      std::unique_lock<std::mutex> channel_lock(*nav_pkt_.mtx);
      nav_pkt_.cv->wait(channel_lock, [this] { return *nav_pkt_.update_complete || !*running_; });
      // nav_pkt_.cv->wait(channel_lock);
      *nav_pkt_.update_complete = false;
    }
  }
}

// *=== Integrate ===*
void ChannelGpsL1caArray::Integrate(const uint64_t &samp_to_read) {
  // std::cout << "ChannelGpsL1caArray::Integrate - W: " << W_(0) << ", " << W_(1) << ", " << W_(2)
  //           << ", " << W_(3) << "\n";
  // get NCO frequencies
  double nco_code_freq = satutils::GPS_CA_CODE_RATE<> + code_doppler_;
  double nco_carr_freq = intmd_freq_rad_ + carr_doppler_;

  if (file_pkt_.TrackingStatus & TrackingFlags::BIT_SYNC) {
    // accumulate samples and beamsteer (Least mean squares)
    LmsBeam(
        mu_,
        W_,
        W_err_,
        shm_->block(shm_ptr_, 0u, samp_to_read, (uint64_t)conf_.rfsignal.n_ant),
        code_.data(),
        rem_code_phase_,
        nco_code_freq,
        rem_carr_phase_,
        nco_carr_freq,
        carr_jitter_,
        conf_.rfsignal.samp_freq,
        half_samp_,
        samp_remaining_,
        tap_space_,
        E_,
        P1_,
        P2_,
        L_);
  } else {
    // accumulate samples
    AccumulateEPL(
        shm_->col(0).segment(shm_ptr_, samp_to_read),
        code_.data(),
        rem_code_phase_,
        nco_code_freq,
        rem_carr_phase_,
        nco_carr_freq,
        carr_jitter_,
        conf_.rfsignal.samp_freq,
        half_samp_,
        samp_remaining_,
        tap_space_,
        E_,
        P1_,
        P2_,
        L_);
  }
  // P_ += (P1_ + P2_);

  // move forward in buffer
  shm_ptr_ += samp_to_read;
  shm_ptr_ %= shm_file_size_samp_;
}

}  // namespace sturdr