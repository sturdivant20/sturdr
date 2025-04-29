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

#include <cmath>
#include <complex>
#include <functional>

#include "sturdr/discriminator.hpp"
#include "sturdr/fftw-wrapper.hpp"
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
    std::shared_ptr<ConcurrentBarrier> barrier1,
    std::shared_ptr<ConcurrentBarrier> barrier2,
    std::shared_ptr<ConcurrentQueue> nav_queue,
    std::shared_ptr<FftwWrapper> fftw_plans,
    std::function<void(uint8_t &)> &GetNewPrnFunc)
    : ChannelGpsL1ca(
          conf, n, running, shared_array, barrier1, barrier2, nav_queue, fftw_plans, GetNewPrnFunc),
      p_array_{Eigen::VectorXcd::Zero(conf.antenna.n_ant)},
      p1_array_{Eigen::VectorXcd::Zero(conf.antenna.n_ant)},
      p2_array_{Eigen::VectorXcd::Zero(conf.antenna.n_ant)},
      e_array_{Eigen::VectorXcd::Zero(conf.antenna.n_ant)},
      l_array_{Eigen::VectorXcd::Zero(conf.antenna.n_ant)},
      w_phase_cal_{Eigen::VectorXcd::Zero(conf.antenna.n_ant)},
      bf_{BeamFormer(conf_.antenna.n_ant, nav_pkt_.Lambda, conf_.antenna.ant_xyz)},
      is_bf_{false} {
  nav_pkt_.PromptCorrelators.resize(conf_.antenna.n_ant);
  nav_pkt_.PromptCorrelators = Eigen::VectorXcd::Zero(conf_.antenna.n_ant);
  nav_pkt_.PllDisc.resize(conf_.antenna.n_ant);
  // w_phase_cal_ << std::complex<double>(1.0, 0.0),
  //     std::complex<double>(1.0, 0.0),
  //     std::complex<double>(1.0, 0.0),
  //     std::complex<double>(1.0, 0.0);
  w_phase_cal_ << std::complex<double>(1.0, 0.0),
      std::complex<double>(-0.948803223566488, -0.315867761808388),
      std::complex<double>(-0.231189814499734, +0.97290866460916),
      std::complex<double>(-0.533946572224442, +0.845518218614932);
}

// *=== ~ChannelGpsL1caArray ===*
ChannelGpsL1caArray::~ChannelGpsL1caArray() {
  // log_->trace("~ChannelGpsL1caArray");
}

// *=== Integrate ===*
void ChannelGpsL1caArray::Integrate(const uint64_t &samp_to_read) {
  double nco_code_freq = satutils::GPS_CA_CODE_RATE<> + code_doppler_;
  double nco_carr_freq = intmd_freq_rad_ + carr_doppler_;

  // accumulate samples
  AccumulateEPLArray(
      shm_->block(shm_ptr_, 0, samp_to_read, (int)conf_.antenna.n_ant),
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
      noise_taps_,
      e_array_,
      p1_array_,
      p2_array_,
      l_array_,
      N_);
  // AccumulateEPLArray(
  //     shm_->block(shm_ptr_, 0, samp_to_read, (int)conf_.antenna.n_ant),
  //     code_.data(),
  //     rem_code_phase_,
  //     nco_code_freq,
  //     rem_carr_phase_,
  //     nco_carr_freq,
  //     carr_jitter_,
  //     conf_.rfsignal.samp_freq,
  //     half_samp_,
  //     samp_remaining_,
  //     tap_space_,
  //     e_array_,
  //     p1_array_,
  //     p2_array_,
  //     l_array_);

  // move forward in buffer
  shm_ptr_ += samp_to_read;
  shm_ptr_ %= shm_file_size_samp_;
}

// *=== Dump ===*
void ChannelGpsL1caArray::Dump() {
  // log_->warn("u_body = [{}, {}, {}]", u_body_(0), u_body_(1), u_body_(2));
  // phase calibrations
  // p1_array_.array() *= w_phase_cal_.array();
  // p2_array_.array() *= w_phase_cal_.array();
  // e_array_.array() *= w_phase_cal_.array();
  // l_array_.array() *= w_phase_cal_.array();

  // beamsteer
  if (!std::isnan((*nav_pkt_.UnitVec)(0))) {
    // if (!is_bf_) {
    //   lock_.Reset();
    //   is_bf_ = true;
    // }
    bf_.CalcSteeringWeights(*nav_pkt_.UnitVec);
    P1_ = bf_(p1_array_);
    P2_ = bf_(p2_array_);
    E_ = bf_(e_array_);
    L_ = bf_(l_array_);
    N_.array() *= static_cast<double>(conf_.antenna.n_ant) / 2.0;
  } else {
    P1_ = p1_array_(0);
    P2_ = p2_array_(0);
    E_ = e_array_(0);
    L_ = l_array_(0);
  }

  // combine prompt sections
  p_array_ = p1_array_ + p2_array_;
  P_ = P1_ + P2_;

  // log_->warn(
  //     "Channel {} - IE = {:.1f}, IP = {:.1f}, IL = {:.1f}",
  //     file_pkt_.Header.ChannelNum,
  //     E_.real() / 10000.0,
  //     P_.real() / 10000.0,
  //     L_.real() / 10000.0);
  // log_->warn(
  //     "Channel {} - QE = {:.1f}, QP = {:.1f}, QL = {:.1f}",
  //     file_pkt_.Header.ChannelNum,
  //     E_.imag() / 10000.0,
  //     P_.imag() / 10000.0,
  //     L_.imag() / 10000.0);

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
  lock_.Update(P_, N_, T_);
  // lock_.Update(P_, T_);
  code_lock_ = lock_.GetCodeLock();
  carr_lock_ = lock_.GetCarrierLock();
  cno_ = lock_.GetCno();
  // double cno_single = cno_ / (double)conf_.antenna.n_ant;

  // discriminators
  double chip_err = DllNneml2(E_, L_);       // [chips]
  double phase_err = PllCostas(P_);          // [rad]
  double freq_err = FllAtan2(P1_, P2_, T_);  // [rad/s]
  double chip_var = DllVariance(cno_, T_);
  // double phase_var = PllVariance(cno_single, T_);
  double phase_var = PllVariance(cno_, T_);
  double freq_var = FllVariance(cno_, T_);

  // update time of week
  file_pkt_.ToW += T_;
  file_pkt_.ToW = std::rint(file_pkt_.ToW * 100.0) / 100.0;
  nav_pkt_.ToW = file_pkt_.ToW;

  // update nav packet
  nav_pkt_.DllDisc = chip_err;
  for (uint8_t i = 0; i < conf_.antenna.n_ant; i++) {
    nav_pkt_.PllDisc(i) = PllAtan2(p_array_(i));
  }
  nav_pkt_.FllDisc = freq_err;
  nav_pkt_.PsrVar = beta_sq_ * chip_var;       // to (m)^2
  nav_pkt_.PsrdotVar = lambda_sq_ * freq_var;  // to (m/s)^2
  nav_pkt_.PhaseVar = 2.0 * phase_var;
  nav_pkt_.PromptCorrelators = p_array_;
  nav_pkt_.CNo = (cno_ > 0.0) ? 10.0 * std::log10(cno_) : 0.0;

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
    *nav_pkt_.update_complete = false;
    q_nav_->push(nav_pkt_);
    {
      std::unique_lock<std::mutex> channel_lock(*nav_pkt_.mtx);
      // log_->warn(
      //     "Channel {}, update_complete = {}, is_vector = {}, file_ptr = {}",
      //     (int)nav_pkt_.Header.ChannelNum,
      //     (int)*nav_pkt_.update_complete,
      //     (int)*nav_pkt_.is_vector,
      //     (int)shm_ptr_);
      nav_pkt_.cv->wait(channel_lock, [this] { return *nav_pkt_.update_complete || !*running_; });
    }
    carr_doppler_ = *nav_pkt_.VTCarrierFreq - intmd_freq_rad_;
    carr_jitter_ = 0.0;
    code_doppler_ = *nav_pkt_.VTCodeRate - satutils::GPS_CA_CODE_RATE<>;

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
  int_per_cnt_ += T_ms_;

  // update file packet
  file_pkt_.Doppler = carr_doppler_ / navtools::TWO_PI<>;
  file_pkt_.CNo = nav_pkt_.CNo;
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
  // file_log_->info("{}", file_pkt_);
  file_log_->write(reinterpret_cast<char *>(&int_per_cnt_), sizeof(uint64_t));
  file_log_->write(reinterpret_cast<char *>(&file_pkt_), sizeof(ChannelPacket));

  // begin next nco period
  NewCodePeriod();
  Status();

  // reset correlators to zero
  p_array_.setZero();
  p1_array_.setZero();
  p2_array_.setZero();
  e_array_.setZero();
  l_array_.setZero();
  P_old_ = P_;
  E_ = 0.0;
  P_ = 0.0;
  L_ = 0.0;
  P1_ = 0.0;
  P2_ = 0.0;
  N_.setZero();
}

}  // namespace sturdr