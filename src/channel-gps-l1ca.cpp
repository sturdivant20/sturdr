/**
 * *channel-gps-l1ca.cpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/channel-gps-l1ca.cpp
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

#include "sturdr/channel-gps-l1ca.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <navtools/constants.hpp>
#include <satutils/code-gen.hpp>
#include <satutils/gnss-constants.hpp>
#include <string>

#include "sturdr/acquisition.hpp"
#include "sturdr/discriminator.hpp"
#include "sturdr/fftw-wrapper.hpp"
#include "sturdr/gnss-signal.hpp"
#include "sturdr/lock-detectors.hpp"
#include "sturdr/structs-enums.hpp"
#include "sturdr/tracking.hpp"

namespace sturdr {

// *=== ChannelGpsL1ca ===*
ChannelGpsL1ca::ChannelGpsL1ca(
    Config &conf,
    uint8_t &n,
    std::shared_ptr<bool> running,
    std::shared_ptr<Eigen::MatrixXcd> shared_array,
    std::shared_ptr<ConcurrentBarrier> barrier1,
    std::shared_ptr<ConcurrentBarrier> barrier2,
    std::shared_ptr<ConcurrentQueue> nav_queue,
    std::shared_ptr<FftwWrapper> fftw_plans,
    std::function<void(uint8_t &)> &GetNewPrnFunc)
    : Channel(
          conf, n, running, shared_array, barrier1, barrier2, nav_queue, fftw_plans, GetNewPrnFunc),
      intmd_freq_rad_{navtools::TWO_PI<> * conf_.rfsignal.intmd_freq},
      rem_code_phase_{0.0},
      code_doppler_{0.0},
      rem_carr_phase_{0.0},
      carr_doppler_{0.0},
      carr_jitter_{0.0},
      cno_{0.0},
      // nbd_{0.0},
      // nbp_{0.0},
      // pc_{0.0},
      // pn_{0.0},
      code_lock_{false},
      carr_lock_{false},
      lock_{conf_.tracking.cno_alpha},
      track_mode_{0u},
      kappa_{satutils::GPS_CA_CODE_RATE<> / (navtools::TWO_PI<> * satutils::GPS_L1_FREQUENCY<>)},
      w0d_{NaturalFrequency(conf_.tracking.dll_bw_wide, 2)},
      w0p_{NaturalFrequency(conf_.tracking.pll_bw_wide, 3)},
      w0f_{NaturalFrequency(conf_.tracking.fll_bw_wide, 2)},
      tap_space_{conf_.tracking.tap_epl_wide},
      noise_taps_{50, 110, 150, 225, 310, 400, 495, 600, 720, 805, 900, 980},
      kf_{TrackingKF()},
      E_{std::complex<double>(0.0, 0.0)},
      P_{std::complex<double>(0.0, 0.0)},
      L_{std::complex<double>(0.0, 0.0)},
      P1_{std::complex<double>(0.0, 0.0)},
      P2_{std::complex<double>(0.0, 0.0)},
      P_old_{std::complex<double>(0.0, 0.0)},
      N_{Eigen::Vector<std::complex<double>, 12>::Zero()},
      T_{0.001},
      T_ms_{1},
      int_per_cnt_{0},
      total_samp_{conf_.acquisition.num_coh_per * conf_.acquisition.num_noncoh_per * samp_per_ms_},
      half_samp_{total_samp_ / 2},
      samp_remaining_{0},
      gps_lnav_{satutils::GpsLnav<double>()} {
  // grab prn to try signal processing with
  file_pkt_.Header.Signal = GnssSignal::GPS_L1CA;
  file_pkt_.Header.Constellation = GnssSystem::GPS;
  new_prn_func_(file_pkt_.Header.SVID);
  nav_pkt_.Header = file_pkt_.Header;
  eph_pkt_.Header = file_pkt_.Header;
  satutils::CodeGenCA(code_.data(), file_pkt_.Header.SVID);
  log_->info(
      "SturDR Channel {} initialized to GPS{}", file_pkt_.Header.ChannelNum, file_pkt_.Header.SVID);

  std::fill(std::begin(bit_sync_hist_), std::end(bit_sync_hist_), 1);

  nav_pkt_.ChipRate = satutils::GPS_CA_CODE_RATE<>;
  nav_pkt_.CarrierFreq = satutils::GPS_L1_FREQUENCY<> * navtools::TWO_PI<>;
  nav_pkt_.Beta = navtools::LIGHT_SPEED<> / nav_pkt_.ChipRate;
  nav_pkt_.Lambda = navtools::LIGHT_SPEED<> / nav_pkt_.CarrierFreq;
  beta_sq_ = nav_pkt_.Beta * nav_pkt_.Beta;
  lambda_sq_ = nav_pkt_.Lambda * nav_pkt_.Lambda;

  nav_pkt_.PllDisc.resize(1);
}

// *=== ~ChannelGpsL1ca ===*
ChannelGpsL1ca::~ChannelGpsL1ca() {
  // log_->trace("~ChannelGpsL1ca");
  file_log_->close();
  if (thread_->joinable()) {
    thread_->join();
  }
}

// *=== NewCodePeriod ===*
void ChannelGpsL1ca::NewCodePeriod() {
  double code_phase_step =
      (satutils::GPS_CA_CODE_RATE<> + code_doppler_) / conf_.rfsignal.samp_freq;
  total_samp_ = static_cast<uint64_t>(std::ceil(
      (static_cast<double>(T_ms_ * satutils::GPS_CA_CODE_LENGTH) - rem_code_phase_) /
      code_phase_step));
  half_samp_ = static_cast<uint64_t>(std::round(0.5 * static_cast<double>(total_samp_)));
  samp_remaining_ = total_samp_;
}

// *=== Acquire ===*
void ChannelGpsL1ca::Acquire() {
  // make sure there are enough samples to acquire with
  if (UnreadSampleCount() < total_samp_) return;

  // Perform parallel acquisition/correlation
  Eigen::MatrixXd corr_map = PcpsSearch(
      *fftw_plans_,
      shm_->col(0).segment(shm_ptr_, total_samp_),
      code_.data(),
      conf_.acquisition.doppler_range,
      conf_.acquisition.doppler_step,
      conf_.rfsignal.samp_freq,
      satutils::GPS_CA_CODE_RATE<>,
      conf_.rfsignal.intmd_freq,
      conf_.acquisition.num_coh_per,
      conf_.acquisition.num_noncoh_per);

  // test for success
  int max_peak_idx[2];
  double metric;
  // GlrtTest(corr_map, max_peak_idx, metric, shm_->segment(shm_ptr_, total_samp_));
  Peak2NoiseFloorTest(corr_map, max_peak_idx, metric);

  if (metric < conf_.acquisition.threshold) {
    // --- FAILURE ---
    shm_ptr_ += total_samp_;
    shm_ptr_ %= shm_file_size_samp_;
    log_->debug(
        "Channel{} failed to acquire GPS{} - Metric: {}",
        file_pkt_.Header.ChannelNum,
        file_pkt_.Header.SVID,
        metric);

    // try a new prn
    new_prn_func_(file_pkt_.Header.SVID);
    satutils::CodeGenCA(code_.data(), file_pkt_.Header.SVID);
    nav_pkt_.Header.SVID = file_pkt_.Header.SVID;
    eph_pkt_.Header.SVID = file_pkt_.Header.SVID;

    // check fail count
    acq_fail_cnt_++;
    if (acq_fail_cnt_ > conf_.acquisition.max_failed_attempts) {
      file_pkt_.ChannelStatus = ChannelState::IDLE;
      return;
    }

    Acquire();

  } else {
    // --- SUCCESS ---
    file_pkt_.ChannelStatus = ChannelState::TRACKING;
    file_pkt_.Doppler =
        -conf_.acquisition.doppler_range + max_peak_idx[1] * conf_.acquisition.doppler_step;
    nav_pkt_.Doppler = carr_doppler_;
    log_->info(
        "{}: GPS{} acquired! Doppler (Hz) = {:.0f}, Code Phase (samp) = {:d}, metric = {:.1f}",
        file_pkt_.Header.ChannelNum,
        file_pkt_.Header.SVID,
        file_pkt_.Doppler,
        max_peak_idx[0],
        metric);

    // update file pointer
    shm_ptr_ += (total_samp_ - samp_per_ms_ + static_cast<uint64_t>(max_peak_idx[0]));
    shm_ptr_ %= shm_file_size_samp_;

    // initialize tracking
    carr_doppler_ = navtools::TWO_PI<> * file_pkt_.Doppler;
    code_doppler_ = kappa_ * carr_doppler_;
    kf_.Init(
        rem_carr_phase_,
        carr_doppler_,
        rem_code_phase_,
        intmd_freq_rad_,
        satutils::GPS_CA_CODE_RATE<>);
    NewCodePeriod();

    // std::string fname = "Channel_" + std::to_string(file_pkt_.Header.ChannelNum) + "_GPS" +
    //                     std::to_string(file_pkt_.Header.SVID);
    // std::ofstream file(fname, std::ios::binary);
    // file.write(reinterpret_cast<char *>(corr_map.data()), corr_map.size() * sizeof(double));
    // file.close();

    // begin tracking
    Track();
  }
}

// *=== Track ===*
void ChannelGpsL1ca::Track() {
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
    }

    // recount unread samples
    unread_samples = UnreadSampleCount();
  }

  // send navigation parameters precise to current sample
  if (!(*nav_pkt_.is_vector)) {
    // log_->info("pushing message to navigator ...");
    nav_pkt_.FilePtr = shm_ptr_;
    nav_pkt_.CodePhase = rem_code_phase_;
    nav_pkt_.Doppler = carr_doppler_;
    nav_pkt_.CarrierPhase = rem_carr_phase_;
    *nav_pkt_.update_complete = false;

    q_nav_->push(nav_pkt_);
    {
      std::unique_lock<std::mutex> channel_lock(*nav_pkt_.mtx);
      nav_pkt_.cv->wait(channel_lock, [this] { return *nav_pkt_.update_complete || !*running_; });
    }

    // log_->trace(
    //     "Channel {}, scalar processing, is_vector = {}, file_ptr = {} ...",
    //     (int)nav_pkt_.Header.ChannelNum,
    //     (int)*nav_pkt_.is_vector,
    //     (int)shm_ptr_);
  }
}

// *=== Integrate ===*
void ChannelGpsL1ca::Integrate(const uint64_t &samp_to_read) {
  // get NCO frequencies
  double nco_code_freq = satutils::GPS_CA_CODE_RATE<> + code_doppler_;
  double nco_carr_freq = intmd_freq_rad_ + carr_doppler_;

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
      noise_taps_,
      E_,
      P1_,
      P2_,
      L_,
      N_);
  // AccumulateEPL(
  //     shm_->col(0).segment(shm_ptr_, samp_to_read),
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
  //     E_,
  //     P1_,
  //     P2_,
  //     L_);
  // P_ += (P1_ + P2_);

  // move forward in buffer
  shm_ptr_ += samp_to_read;
  shm_ptr_ %= shm_file_size_samp_;
}

// *=== Dump ===*
void ChannelGpsL1ca::Dump() {
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
  lock_.Update(P_, N_, T_);
  // lock_.Update(P_, T_);
  code_lock_ = lock_.GetCodeLock();
  carr_lock_ = lock_.GetCarrierLock();
  cno_ = lock_.GetCno();

  // discriminators
  double chip_err = DllNneml2(E_, L_);       // [chips]
  double phase_err = PllCostas(P_);          // [rad]
  double freq_err = FllAtan2(P1_, P2_, T_);  // [rad/s]
  double chip_var = DllVariance(cno_, T_);
  double phase_var = PllVariance(cno_, T_);
  double freq_var = FllVariance(cno_, T_);

  // update time of week
  file_pkt_.ToW += T_;
  file_pkt_.ToW = std::rint(file_pkt_.ToW * 100) / 100;
  nav_pkt_.ToW = file_pkt_.ToW;

  // update nav packet
  nav_pkt_.DllDisc = chip_err;
  nav_pkt_.PllDisc(0) = phase_err;
  nav_pkt_.FllDisc = freq_err;
  nav_pkt_.PsrVar = beta_sq_ * chip_var;
  nav_pkt_.PsrdotVar = lambda_sq_ * freq_var;
  nav_pkt_.PhaseVar = phase_var;
  nav_pkt_.CNo = (cno_ > 0.0) ? 10.0 * std::log10(cno_) : 0.0;

  // log_->trace(
  //     "Channel {}, scalar processing, is_vector = {}, file_ptr = {} ...",
  //     (int)nav_pkt_.Header.ChannelNum,
  //     (int)*nav_pkt_.is_vector,
  //     (int)shm_ptr_);
  // tracking loop
  if (!*(nav_pkt_.is_vector)) {
    // *--- scalar process ---*
    // std::cout << "Channel " << (int)nav_pkt_.Header.ChannelNum
    //           << ", is_vector = " << *nav_pkt_.is_vector << "\n";
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
    // log_->trace(
    //     "Channel {}, scalar processing, is_vector = {}, file_ptr = {} ...",
    //     (int)nav_pkt_.Header.ChannelNum,
    //     (int)*nav_pkt_.is_vector,
    //     (int)shm_ptr_);
  } else {
    // *--- vector process ---
    // log_->warn(
    //     "Channel {}, is_vector = {}", (int)nav_pkt_.Header.ChannelNum, (int)*nav_pkt_.is_vector);

    rem_carr_phase_ = std::fmod(rem_carr_phase_, navtools::TWO_PI<>);
    rem_code_phase_ -= static_cast<double>(T_ms_ * satutils::GPS_CA_CODE_LENGTH);

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

  // no need to log additional correlators here
  // file_log_->info("{}", file_pkt_);
  file_log_->write(reinterpret_cast<char *>(&int_per_cnt_), sizeof(uint64_t));
  file_log_->write(
      reinterpret_cast<char *>(&file_pkt_), sizeof(ChannelPacket) - 8 * sizeof(double));

  // begin next nco period
  NewCodePeriod();
  Status();

  // reset correlators to zero
  P_old_ = P_;
  E_ = 0.0;
  P_ = 0.0;
  L_ = 0.0;
  P1_ = 0.0;
  P2_ = 0.0;
  N_.setZero();
}

// *=== Status ===*
void ChannelGpsL1ca::Status() {
  // update tracking status
  if (code_lock_) {
    file_pkt_.TrackingStatus |= TrackingFlags::CODE_LOCK;
  } else {
    file_pkt_.TrackingStatus &= ~TrackingFlags::CODE_LOCK;
  }
  if (carr_lock_) {
    file_pkt_.TrackingStatus |= TrackingFlags::CARRIER_LOCK;
  } else {
    file_pkt_.TrackingStatus &= ~TrackingFlags::CARRIER_LOCK;
  }

  if (!(*nav_pkt_.is_vector)) {
    // mode 0: wide tracking - only check code lock and increment stage by 1
    if (track_mode_ == 0) {
      if (code_lock_) {
        track_mode_ = 1;
        w0d_ = NaturalFrequency(conf_.tracking.dll_bw_standard, 2);
        w0p_ = NaturalFrequency(conf_.tracking.pll_bw_standard, 3);
        w0f_ = NaturalFrequency(conf_.tracking.fll_bw_standard, 2);
        tap_space_ = conf_.tracking.tap_epl_standard;
      }
    }

    // mode 1: normal tracking
    if (track_mode_ == 1) {
      if (code_lock_ & carr_lock_) {
        track_mode_ = 2;
        w0d_ = NaturalFrequency(conf_.tracking.dll_bw_narrow, 2);
        w0p_ = NaturalFrequency(conf_.tracking.pll_bw_narrow, 3);
        w0f_ = NaturalFrequency(conf_.tracking.fll_bw_narrow, 2);
        tap_space_ = conf_.tracking.tap_epl_narrow;
        file_pkt_.TrackingStatus |= TrackingFlags::FINE_LOCK;
      } else if (!code_lock_) {
        track_mode_ = 0;
        w0d_ = NaturalFrequency(conf_.tracking.dll_bw_wide, 2);
        w0p_ = NaturalFrequency(conf_.tracking.pll_bw_wide, 3);
        w0f_ = NaturalFrequency(conf_.tracking.fll_bw_wide, 2);
        tap_space_ = conf_.tracking.tap_epl_wide;
      }
    }

    // mode 2: narrow tracking
    if (track_mode_ == 2) {
      if (!(code_lock_ & carr_lock_)) {
        track_mode_ = 1;
        w0d_ = NaturalFrequency(conf_.tracking.dll_bw_standard, 2);
        w0p_ = NaturalFrequency(conf_.tracking.pll_bw_standard, 3);
        w0f_ = NaturalFrequency(conf_.tracking.fll_bw_standard, 2);
        tap_space_ = conf_.tracking.tap_epl_standard;
        file_pkt_.TrackingStatus &= ~TrackingFlags::FINE_LOCK;
      }
    }
  }
}

// *=== NavDataSync ===*
bool ChannelGpsL1ca::NavDataSync() {
  // check for a bit flip
  if (!((P_.real() < 0.0) == (P_old_.real() < 0.0))) {
    // data bit flip!
    bit_sync_hist_[int_per_cnt_ % 20]++;
    uint8_t sorted_bit_sync_hist[20];
    std::copy(std::begin(bit_sync_hist_), std::end(bit_sync_hist_), sorted_bit_sync_hist);
    std::sort(std::begin(sorted_bit_sync_hist), std::end(sorted_bit_sync_hist));

    if (int_per_cnt_ > (uint64_t)conf_.tracking.min_converg_time_ms) {
      // check for data lock
      if (sorted_bit_sync_hist[19] >= (4 * sorted_bit_sync_hist[18])) {
        log_->debug(
            "SturDR Channel {}: GPS{} data bit sync! cnt_ = {}",
            file_pkt_.Header.ChannelNum,
            file_pkt_.Header.SVID,
            int_per_cnt_);

        // we have found a data bit to align to -> immediately continue last 19ms of this bit
        T_ = 0.02;
        T_ms_ = 20;
        P1_ += P2_;
        P2_ = 0.0;
        // NewCodePeriod();
        uint64_t tmp_samp = total_samp_;
        double code_phase_step =
            (satutils::GPS_CA_CODE_RATE<> + code_doppler_) / conf_.rfsignal.samp_freq;
        total_samp_ = static_cast<uint64_t>(std::ceil(
            (static_cast<double>(T_ms_ * satutils::GPS_CA_CODE_LENGTH) - file_pkt_.CodePhase) /
            code_phase_step));
        half_samp_ = static_cast<uint64_t>(std::round(0.5 * static_cast<double>(total_samp_)));
        samp_remaining_ = total_samp_ - tmp_samp;
        file_pkt_.TrackingStatus |= TrackingFlags::BIT_SYNC;
        return true;

      } else if (std::any_of(std::begin(bit_sync_hist_), std::end(bit_sync_hist_), [](int num) {
                   return num > 5;
                 })) {
        // reset occasionally to make it plausible for the bit sync to succeed
        std::fill(std::begin(bit_sync_hist_), std::end(bit_sync_hist_), 1);
      }
    }
  }
  return false;
}

// *=== Demodulate ===*
void ChannelGpsL1ca::Demodulate() {
  // give next bit to LNAV-PARSER
  bool bit = P_.real() >= 0;
  if (gps_lnav_.SetNextBit(bit)) {
    // if here - a subframe has been parsed
    file_pkt_.Week = gps_lnav_.GetWeekNumber();
    file_pkt_.ToW = gps_lnav_.GetTimeOfWeek() - 0.02;
    file_pkt_.ToW = std::rint(file_pkt_.ToW * 100) / 100;
    nav_pkt_.Week = file_pkt_.Week;
    nav_pkt_.ToW = file_pkt_.ToW;
    file_pkt_.TrackingStatus |= TrackingFlags::TOW_DECODED;
    file_pkt_.TrackingStatus |= TrackingFlags::SUBFRAME_SYNC;
    // log_->info(
    //     "SturDR Channel {}: GPS{} parsed a subframe!",
    //     eph_pkt_.Header.ChannelNum,
    //     eph_pkt_.Header.SVID);
  }

  // check if all subframes have been parsed
  if (gps_lnav_.AreEphemeridesParsed()) {
    eph_pkt_.Header = file_pkt_.Header;
    eph_pkt_.Eph = gps_lnav_.GetEphemerides();
    q_nav_->push(eph_pkt_);
    log_->debug(
        "SturDR Channel {}: GPS{} ephemeris subframes 1,2 and 3 parsed!",
        eph_pkt_.Header.ChannelNum,
        eph_pkt_.Header.SVID);
    file_pkt_.TrackingStatus |= TrackingFlags::EPH_DECODED;
  }
}

}  // namespace sturdr