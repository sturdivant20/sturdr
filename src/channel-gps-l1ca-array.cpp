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
#include "sturdr/discriminator.hpp"
#include "sturdr/gnss-signal.hpp"
#include "sturdr/lock-detectors.hpp"
#include "sturdr/structs-enums.hpp"

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
      W_{Eigen::VectorXcd::Zero(conf_.antenna.n_ant)} {
  W_(0) = 1.0;
  nav_pkt_.PromptCorrelators.resize(conf_.antenna.n_ant);
  nav_pkt_.PromptCorrelators = Eigen::VectorXcd::Zero(conf_.antenna.n_ant);
  nav_pkt_.PllDisc.resize(conf_.antenna.n_ant);
}

// *=== ~ChannelGpsL1caArray ===*
ChannelGpsL1caArray::~ChannelGpsL1caArray() {
}

// // *=== Track ===*
// void ChannelGpsL1caArray::Track() {
//   // make sure a count of the unprocessed samples is made
//   uint64_t unread_samples = UnreadSampleCount();
//   uint64_t samp_to_read;

//   // process until all samples have been read
//   while (unread_samples > 0) {
//     if (samp_remaining_ > 0) {
//       // --- INTEGRATE ---
//       samp_to_read = std::min(samp_remaining_, unread_samples);
//       // std::cout << "samp_to_read: " << samp_to_read << ", samp_remaining: " << samp_remaining_
//       //           << ", unread_samples: " << unread_samples << "\n";
//       Integrate(samp_to_read);

//     } else {
//       // --- DUMP ---
//       Dump();
//     }

//     // recount unread samples
//     unread_samples = UnreadSampleCount();
//   }

//   // send navigation parameters precise to current sample
//   if (!(*nav_pkt_.is_vector)) {
//     nav_pkt_.FilePtr = shm_ptr_;
//     nav_pkt_.CodePhase = rem_code_phase_;
//     nav_pkt_.Doppler = carr_doppler_;
//     nav_pkt_.CarrierPhase = rem_carr_phase_;
//     q_nav_->push(nav_pkt_);
//     {
//       std::unique_lock<std::mutex> channel_lock(*nav_pkt_.mtx);
//       nav_pkt_.cv->wait(channel_lock, [this] { return *nav_pkt_.update_complete || !*running_;
//       });
//       // nav_pkt_.cv->wait(channel_lock);
//       *nav_pkt_.update_complete = false;
//     }
//   }
// }

// *=== Integrate ===*
void ChannelGpsL1caArray::Integrate(const uint64_t &samp_to_read) {
  double nco_code_freq = satutils::GPS_CA_CODE_RATE<> + code_doppler_;
  double nco_carr_freq = intmd_freq_rad_ + carr_doppler_;

  // if (*nav_pkt_.is_vector) {
  //   DeterministicBeam(
  //       conf_.antenna.ant_xyz,
  //       u_,
  //       shm_->block(shm_ptr_, 0u, samp_to_read, (uint64_t)conf_.antenna.n_ant),
  //       code_.data(),
  //       rem_code_phase_,
  //       nco_code_freq,
  //       rem_carr_phase_,
  //       nco_carr_freq,
  //       carr_jitter_,
  //       conf_.rfsignal.samp_freq,
  //       half_samp_,
  //       samp_remaining_,
  //       tap_space_,
  //       E_,
  //       P1_,
  //       P2_,
  //       L_);
  //   for (int i = 0; i < conf_.antenna.n_ant; i++) {
  //     Correlate(
  //         shm_->col(i).segment(shm_ptr_, samp_to_read),
  //         code_.data(),
  //         rem_code_phase_,
  //         nco_code_freq,
  //         rem_carr_phase_,
  //         nco_carr_freq,
  //         carr_jitter_,
  //         conf_.rfsignal.samp_freq,
  //         nav_pkt_.PromptCorrelators(i));
  //   }
  // } else {
  double init_code_phase = rem_code_phase_, init_carr_phase = rem_carr_phase_;
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

  nav_pkt_.PromptCorrelators(0) += (P1_ + P2_);
  double tmp_code_phase, tmp_carr_phase;
  for (int i = 1; i < conf_.antenna.n_ant; i++) {
    tmp_code_phase = init_code_phase;
    tmp_carr_phase = init_carr_phase;
    Correlate(
        shm_->col(i).segment(shm_ptr_, samp_to_read),
        code_.data(),
        tmp_code_phase,
        nco_code_freq,
        tmp_carr_phase,
        nco_carr_freq,
        carr_jitter_,
        conf_.rfsignal.samp_freq,
        nav_pkt_.PromptCorrelators(i));
  }
  // }
  // P_ += (P1_ + P2_);

  // move forward in buffer
  shm_ptr_ += samp_to_read;
  shm_ptr_ %= shm_file_size_samp_;
}

// *=== Dump ===*
void ChannelGpsL1caArray::Dump() {
  // combine prompt sections
  P_ = P1_ + P2_;

  // check if navigation data needs to be parsed
  if (!(file_pkt_.TrackingStatus & TrackingFlags::EPH_DECODED)) {
    // ephemeris is not decoded
    if ((file_pkt_.TrackingStatus & TrackingFlags::BIT_SYNC)) {
      // already synchronized to data bits, attempt demodulation
      Demodulate();
    } else {
      // attempt to synchronize to data bits
      if (NavDataSync()) {
        return;
      }
    }
  }

  // lock detectors
  LockDetectors(code_lock_, carr_lock_, cno_, nbd_, nbp_, pc_, pn_, P_old_, P_, T_, 0.05);

  // discriminators
  double chip_err = DllNneml2(E_, L_);       // [chips]
  double phase_err = PllCostas(P_);          // [rad]
  double freq_err = FllAtan2(P1_, P2_, T_);  // [rad/s]
  double chip_var = DllVariance(cno_, T_);
  double phase_var = PllVariance(cno_, T_);
  double freq_var = FllVariance(cno_, T_);

  // update time of week
  file_pkt_.ToW += T_;
  nav_pkt_.ToW = file_pkt_.ToW;

  // update nav packet
  nav_pkt_.DllDisc = chip_err;
  // nav_pkt_.PllDisc = phase_err;
  nav_pkt_.FllDisc = freq_err;
  nav_pkt_.PsrVar = beta_sq_ * chip_var;
  nav_pkt_.PsrdotVar = lambda_sq_ * freq_var;
  nav_pkt_.PhaseVar = 2.0 * phase_var;
  for (uint8_t i = 0; i < conf_.antenna.n_ant; i++) {
    nav_pkt_.PllDisc(i) = PllAtan2(nav_pkt_.PromptCorrelators(i));
  }

  // tracking loop
  if (!*(nav_pkt_.is_vector)) {
    // *--- scalar process ---*
    double t = static_cast<double>(total_samp_) / conf_.rfsignal.samp_freq;
    kf_.UpdateDynamicsParam(w0d_, w0p_, w0f_, kappa_, t);
    kf_.UpdateMeasurementsParam(chip_var, phase_var, freq_var);
    kf_.Run(chip_err, phase_err, freq_err);
    rem_carr_phase_ = std::fmod(kf_.x_(0), navtools::TWO_PI<>);
    carr_doppler_ = kf_.x_(1);
    carr_jitter_ = kf_.x_(2);
    rem_code_phase_ = kf_.x_(3) - static_cast<double>(T_ms_ * satutils::GPS_CA_CODE_LENGTH);
    code_doppler_ = kf_.x_(4) + kappa_ * (carr_doppler_ + carr_jitter_ * t);
    kf_.SetRemCarrierPhase(rem_carr_phase_);
    kf_.SetRemCodePhase(rem_code_phase_);
  } else {
    rem_carr_phase_ = std::fmod(rem_carr_phase_, navtools::TWO_PI<>);
    rem_code_phase_ -= static_cast<double>(T_ms_ * satutils::GPS_CA_CODE_LENGTH);

    // *--- vector process ---
    nav_pkt_.FilePtr = shm_ptr_;
    nav_pkt_.CodePhase = rem_code_phase_;
    nav_pkt_.Doppler = carr_doppler_;
    nav_pkt_.CarrierPhase = rem_carr_phase_;
    q_nav_->push(nav_pkt_);
    {
      std::unique_lock<std::mutex> channel_lock(*nav_pkt_.mtx);
      nav_pkt_.cv->wait(channel_lock, [this] { return *nav_pkt_.update_complete || !*running_; });
      *nav_pkt_.update_complete = false;
    }
    carr_doppler_ = *nav_pkt_.VTCarrierFreq - intmd_freq_rad_;
    carr_jitter_ = 0.0;
    code_doppler_ = *nav_pkt_.VTCodeRate - satutils::GPS_CA_CODE_RATE<>;
    // u_ = *nav_pkt_.VTUnitVec;

    // double t = static_cast<double>(total_samp_) / conf_.rfsignal.samp_freq;
    // kf_.UpdateDynamicsParam(w0d_, w0p_, w0f_, kappa_, t);
    // kf_.UpdateMeasurementsParam(chip_var, phase_var, freq_var);
    // kf_.Run(chip_err, phase_err, *nav_pkt_.VTCarrierFreq);
    // rem_carr_phase_ = std::fmod(kf_.x_(0), navtools::TWO_PI<>);
    // carr_doppler_ = kf_.x_(1);
    // carr_jitter_ = kf_.x_(2);
    // // rem_code_phase_ = kf_.x_(3) - static_cast<double>(T_ms_ * satutils::GPS_CA_CODE_LENGTH);
    // // code_doppler_ = kf_.x_(4) + kappa_ * (carr_doppler_ + carr_jitter_ * t);
    // kf_.SetRemCarrierPhase(rem_carr_phase_);
    // kf_.SetRemCodePhase(rem_code_phase_);
  }

  // update file packet
  file_pkt_.Doppler = carr_doppler_ / navtools::TWO_PI<>;
  file_pkt_.CNo = (cno_ > 0.0) ? 10.0 * std::log10(cno_) : 0.0;
  file_pkt_.DllDisc = chip_err;
  file_pkt_.PllDisc = phase_err;
  file_pkt_.FllDisc = freq_err;
  file_pkt_.IE = E_.real();
  file_pkt_.IP = P_.real();
  file_pkt_.IL = L_.real();
  file_pkt_.QE = E_.imag();
  file_pkt_.QP = P_.imag();
  file_pkt_.QL = L_.imag();
  file_pkt_.IP1 = P2_.real();
  file_pkt_.IP2 = P1_.real();
  file_pkt_.QP1 = P1_.imag();
  file_pkt_.QP2 = P2_.imag();
  file_pkt_.CodePhase = rem_code_phase_;
  file_pkt_.CarrierPhase = rem_carr_phase_;
  switch (conf_.antenna.n_ant) {
    case 2:
      file_pkt_.IP_A0 = nav_pkt_.PromptCorrelators(0).real();
      file_pkt_.IP_A1 = nav_pkt_.PromptCorrelators(1).real();
      file_pkt_.QP_A0 = nav_pkt_.PromptCorrelators(0).imag();
      file_pkt_.QP_A1 = nav_pkt_.PromptCorrelators(1).imag();
      break;
    case 3:
      file_pkt_.IP_A0 = nav_pkt_.PromptCorrelators(0).real();
      file_pkt_.IP_A1 = nav_pkt_.PromptCorrelators(1).real();
      file_pkt_.IP_A2 = nav_pkt_.PromptCorrelators(2).real();
      file_pkt_.QP_A0 = nav_pkt_.PromptCorrelators(0).imag();
      file_pkt_.QP_A1 = nav_pkt_.PromptCorrelators(1).imag();
      file_pkt_.QP_A2 = nav_pkt_.PromptCorrelators(2).imag();
      break;
    case 4:
      file_pkt_.IP_A0 = nav_pkt_.PromptCorrelators(0).real();
      file_pkt_.IP_A1 = nav_pkt_.PromptCorrelators(1).real();
      file_pkt_.IP_A2 = nav_pkt_.PromptCorrelators(2).real();
      file_pkt_.IP_A3 = nav_pkt_.PromptCorrelators(3).real();
      file_pkt_.QP_A0 = nav_pkt_.PromptCorrelators(0).imag();
      file_pkt_.QP_A1 = nav_pkt_.PromptCorrelators(1).imag();
      file_pkt_.QP_A2 = nav_pkt_.PromptCorrelators(2).imag();
      file_pkt_.QP_A3 = nav_pkt_.PromptCorrelators(3).imag();
      break;
  }

  // std::cout << "ChannelGpsL1ca::Dump - file log called\n";
  file_log_->info("{}", file_pkt_);

  // begin next nco period
  int_per_cnt_ += T_ms_;
  NewCodePeriod();
  Status();

  // reset correlators to zero
  P_old_ = P_;
  E_ = 0.0;
  P_ = 0.0;
  L_ = 0.0;
  P1_ = 0.0;
  P2_ = 0.0;
  for (int i = 0; i < conf_.antenna.n_ant; i++) {
    nav_pkt_.PromptCorrelators(i) = 0.0;
  }
}

}  // namespace sturdr