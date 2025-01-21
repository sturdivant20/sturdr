/**
 * *channel-gps-l1ca.cpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/channel-gps-l1ca.cpp
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

#include "sturdr/channel-gps-l1ca.hpp"

#include <algorithm>
#include <cstring>
#include <navtools/constants.hpp>
#include <satutils/code-gen.hpp>
#include <satutils/gnss-constants.hpp>

#include "sturdr/acquisition.hpp"
#include "sturdr/discriminator.hpp"
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
    std::shared_ptr<Eigen::VectorXcd> shared_array,
    std::shared_ptr<ConcurrentBarrier> start_barrier,
    std::shared_ptr<ConcurrentBarrier> end_barrier,
    std::shared_ptr<ConcurrentQueue<ChannelEphemPacket>> eph_queue,
    std::shared_ptr<ConcurrentQueue<ChannelNavPacket>> nav_queue,
    FftPlans &fftw_plans,
    std::function<void(uint8_t &)> &GetNewPrnFunc)
    : Channel(
          conf,
          n,
          running,
          shared_array,
          start_barrier,
          end_barrier,
          eph_queue,
          nav_queue,
          fftw_plans,
          GetNewPrnFunc),
      intmd_freq_rad_{navtools::TWO_PI<> * conf_.rfsignal.intmd_freq},
      rem_code_phase_{0.0},
      code_doppler_{0.0},
      rem_carr_phase_{0.0},
      carr_doppler_{0.0},
      carr_jitter_{0.0},
      cno_{0.0},
      nbd_{0.0},
      nbp_{0.0},
      pc_{0.0},
      pn_{0.0},
      code_lock_{false},
      carr_lock_{false},
      track_mode_{0u},
      kappa_{satutils::GPS_CA_CODE_RATE<> / (navtools::TWO_PI<> * satutils::GPS_L1_FREQUENCY<>)},
      w0d_{NaturalFrequency(conf_.tracking.dll_bw_wide, 2)},
      w0p_{NaturalFrequency(conf_.tracking.pll_bw_wide, 3)},
      w0f_{NaturalFrequency(conf_.tracking.fll_bw_wide, 2)},
      tap_space_{conf_.tracking.tap_epl_wide},
      kf_{TrackingKF()},
      E_{std::complex<double>(0.0, 0.0)},
      P_{std::complex<double>(0.0, 0.0)},
      L_{std::complex<double>(0.0, 0.0)},
      P1_{std::complex<double>(0.0, 0.0)},
      P2_{std::complex<double>(0.0, 0.0)},
      P_old_{std::complex<double>(0.0, 0.0)},
      T_{0.001},
      half_T_{0.5 * T_},
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

  std::fill(std::begin(bit_sync_hist_), std::end(bit_sync_hist_), 0);

  beta_ = navtools::LIGHT_SPEED<> / satutils::GPS_CA_CODE_RATE<>;
  lambda_ = navtools::LIGHT_SPEED<> / (satutils::GPS_L1_FREQUENCY<> * navtools::TWO_PI<>);
  beta_sq_ = beta_ * beta_;
  lambda_sq_ = lambda_ * lambda_;
}

// *=== ~ChannelGpsL1ca ===*
ChannelGpsL1ca::~ChannelGpsL1ca() {
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
      fftw_plans_,
      shm_->segment(shm_ptr_, total_samp_),
      code_,
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
  GlrtTest(corr_map, max_peak_idx, metric, shm_->segment(shm_ptr_, total_samp_));

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
        -conf_.acquisition.doppler_range + max_peak_idx[0] * conf_.acquisition.doppler_step;
    nav_pkt_.Psrdot = -lambda_ * carr_doppler_;
    log_->info(
        "{}: GPS{} acquired! Doppler (Hz) = {:.0f}, Code Phase (samp) = {:d}, metric = {:.1f}",
        file_pkt_.Header.ChannelNum,
        file_pkt_.Header.SVID,
        file_pkt_.Doppler,
        max_peak_idx[1],
        metric);

    // update file pointer
    shm_ptr_ += (total_samp_ - samp_per_ms_ + static_cast<uint64_t>(max_peak_idx[1]));
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
    file_pkt_.TrackingStatus |= TrackingFlags::ACQUIRED;

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

  // save intermediate remainder phases for precise navigation
  nav_pkt_.CodePhaseSec = rem_code_phase_ / satutils::GPS_CA_CODE_RATE<>;
  nav_pkt_.Psrdot = -lambda_ * carr_doppler_;
  nav_pkt_.CarrierPhase = rem_carr_phase_;
}

// *=== Integrate ===*
void ChannelGpsL1ca::Integrate(const uint64_t &samp_to_read) {
  // get NCO frequencies
  double nco_code_freq = satutils::GPS_CA_CODE_RATE<> + code_doppler_;
  double nco_carr_freq = intmd_freq_rad_ + carr_doppler_;

  // accumulate samples
  AccumulateEPL(
      shm_->segment(shm_ptr_, samp_to_read),
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
  P_ += (P1_ + P2_);

  // move forward in buffer
  shm_ptr_ += samp_to_read;
  shm_ptr_ %= shm_file_size_samp_;
}

// *=== Dump ===*
void ChannelGpsL1ca::Dump() {
  // check if navigation data needs to be parsed
  if (!(file_pkt_.TrackingStatus & TrackingFlags::EPH_DECODED)) {
    // ephemeris is not decoded
    if ((file_pkt_.TrackingStatus & TrackingFlags::BIT_SYNC)) {
      // already synchronized to data bits, attempt demodulation
      Demodulate();
    } else {
      // attempt to synchronize to data bits
      if (NavDataSync()) return;
    }
  }

  // lock detectors
  LockDetectors(code_lock_, carr_lock_, cno_, nbd_, nbp_, pc_, pn_, P_old_, P_, T_);

  // discriminators
  double chip_err = DllNneml2(E_, L_);            // [chips]
  double phase_err = PllCostas(P_);               // [rad]
  double freq_err = FllAtan2(P1_, P2_, half_T_);  // [rad/s]
  double chip_var = DllVariance(cno_, T_);
  double phase_var = PllVariance(cno_, T_);
  double freq_var = FllVariance(cno_, T_);

  // tracking loop
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

  // update file packet
  file_pkt_.ToW += T_;
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
  file_log_->info("{}", file_pkt_);

  // update nav packet
  nav_pkt_.ToW = file_pkt_.ToW;
  nav_pkt_.CNo = cno_;
  nav_pkt_.DllDisc = file_pkt_.DllDisc;
  nav_pkt_.PllDisc = file_pkt_.PllDisc;
  nav_pkt_.FllDisc = file_pkt_.FllDisc;
  nav_pkt_.PsrVar = beta_sq_ * chip_var;
  nav_pkt_.PsrdotVar = lambda_sq_ * freq_var;

  // begin next nco period
  int_per_cnt_ += T_ms_;
  NewCodePeriod();

  // reset correlators to zero
  P_old_ = P_;
  E_ = 0.0;
  P_ = 0.0;
  L_ = 0.0;
  P1_ = 0.0;
  P2_ = 0.0;
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
      if (sorted_bit_sync_hist[19] > (4 * sorted_bit_sync_hist[18])) {
        log_->info(
            "SturDR Channel {}: GPS{} data bit sync! cnt_ = {}",
            file_pkt_.Header.ChannelNum,
            file_pkt_.Header.SVID,
            int_per_cnt_);

        // we have found a data bit to align to -> immediately continue last 19ms of this bit
        T_ = 0.02;
        T_ms_ = 20;
        P1_ += P2_;
        P2_ = 0.0;
        NewCodePeriod();
        file_pkt_.TrackingStatus |= TrackingFlags::BIT_SYNC;

        // TODO: make this dynamically happen
        tap_space_ = conf_.tracking.tap_epl_narrow;
        w0d_ = NaturalFrequency(conf_.tracking.dll_bw_narrow, 2);
        w0p_ = NaturalFrequency(conf_.tracking.pll_bw_narrow, 3);
        w0f_ = NaturalFrequency(conf_.tracking.fll_bw_narrow, 2);
        return true;
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
    nav_pkt_.Week = file_pkt_.Week;
    nav_pkt_.ToW = file_pkt_.ToW;
    file_pkt_.TrackingStatus |= TrackingFlags::TOW_DECODED;
  }

  // check if all subframes have been parsed
  if (gps_lnav_.AreEphemeridesParsed()) {
    eph_pkt_.Header = file_pkt_.Header;
    eph_pkt_.Eph = gps_lnav_.GetEphemerides();
    q_eph_->push(eph_pkt_);
    log_->info(
        "SturDR Channel {}: GPS{} ephemeris subframes 1,2 and 3 parsed!",
        eph_pkt_.Header.ChannelNum,
        eph_pkt_.Header.SVID);
    file_pkt_.TrackingStatus |= TrackingFlags::EPH_DECODED;
  }
}

}  // namespace sturdr