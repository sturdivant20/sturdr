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

#include <Eigen/Dense>
#include <cassert>
#include <cmath>
#include <exception>
#include <navtools/binary-ops.hpp>
#include <navtools/constants.hpp>
#include <satutils/code-gen.hpp>
#include <satutils/gnss-constants.hpp>

#include "sturdr/acquisition.hpp"
#include "sturdr/discriminator.hpp"
#include "sturdr/gnss-signal.hpp"
#include "sturdr/lock-detectors.hpp"
#include "sturdr/tracking.hpp"

namespace sturdr {

//! ------------------------------------------------------------------------------------------------

// *=== GpsL1caChannel ===*
GpsL1caChannel::GpsL1caChannel(
    const Config &config,
    const AcquisitionSetup &acq_setup,
    std::shared_ptr<Eigen::VectorXcd> shm,
    std::shared_ptr<ConcurrentQueue<NavPacket>> nav_queue,
    std::shared_ptr<ConcurrentBarrier> start_barrier,
    std::shared_ptr<ConcurrentBarrier> end_barrier,
    int &channel_num,
    std::shared_ptr<bool> still_running,
    void (*GetNewPrnFunc)(uint8_t &))
    : Channel(
          config,
          acq_setup,
          shm,
          nav_queue,
          start_barrier,
          end_barrier,
          channel_num,
          still_running,
          GetNewPrnFunc),
      rem_code_phase_{0.0},
      code_doppler_{0.0},
      rem_carr_phase_{0.0},
      carr_doppler_{0.0},
      carr_jitter_{0.0},
      T_{0.001},
      PDI_{1},
      total_samp_{0},
      half_samp_{0},
      samp_processed_{0},
      E_{std::complex<double>(0.0, 0.0)},
      P_{std::complex<double>(0.0, 0.0)},
      L_{std::complex<double>(0.0, 0.0)},
      P1_{std::complex<double>(0.0, 0.0)},
      P2_{std::complex<double>(0.0, 0.0)},
      P_old_{std::complex<double>(0.0, 0.0)},
      track_mode_{0},
      cno_mag_{0.0},
      nbd_mem_{0.0},
      nbp_mem_{0.0},
      pc_mem_{0.0},
      pn_mem_{0.0},
      code_lock_{false},
      carr_lock_{false},
      cnt_{0},
      bit_sync_hist_{0} {
  // initialize constants inside messages
  channel_msg_.Header.ChannelNum = channel_num;
  channel_msg_.Header.Constellation = GnssSystem::GPS;
  channel_msg_.Header.Signal = GnssSignal::GPS_L1CA;
  channel_msg_.ChannelStatus = ChannelState::IDLE;
  nav_msg_.Header.ChannelNum = channel_num;
  nav_msg_.Header.Constellation = GnssSystem::GPS;
  nav_msg_.Header.Signal = GnssSignal::GPS_L1CA;

  // sample constants
  samp_per_ms_ = static_cast<int>(conf_.rfsignal.samp_freq / 1000.0);
  samp_per_chip_ = static_cast<int>(conf_.rfsignal.samp_freq / satutils::GPS_CA_CODE_RATE<>);

  // Tracking filter
  kappa_ = satutils::GPS_CA_CODE_RATE<> / (navtools::TWO_PI<> * satutils::GPS_L1_FREQUENCY<>);
  w0d_ = NaturalFrequency(conf_.tracking.dll_bw_wide, 2);
  w0p_ = NaturalFrequency(conf_.tracking.pll_bw_wide, 3);
  w0f_ = NaturalFrequency(conf_.tracking.fll_bw_wide, 2);
  tap_space_ = conf_.tracking.tap_epl_wide;
}

// *=== ~GpsL1caChannel ===*
GpsL1caChannel::~GpsL1caChannel() {
}

// *=== SetSatellite ===*
void GpsL1caChannel::SetSatellite(uint8_t sv_id) {
  try {
    // update channel status
    channel_msg_.Header.SVID = sv_id;
    channel_msg_.ChannelStatus = ChannelState::ACQUIRING;
    nav_msg_.Header.SVID = sv_id;

    // generate requested code
    satutils::CodeGenCA(code_, sv_id);

    // initialize acquisition sample parameters
    total_samp_ = conf_.acquisition.num_coh_per * conf_.acquisition.num_noncoh_per * samp_per_ms_;
    half_samp_ = total_samp_ / 2;
  } catch (std::exception &e) {
    log_->error("gps-l1ca-channel.cpp SetSatellite failed! Error -> {}", e.what());
  }
}

// // *=== Run ===*
// void GpsL1caChannel::run() {
//   // wait for shm_ to be updated
//   start_bar_->Wait();

//   while (*still_running_) {
//     // update shm_ writer buffer (necessary because writer is a different process)
//     UpdateShmWriterPtr();

//     // process data
//     if (channel_msg_.ChannelStatus == ChannelState::TRACKING) {
//       Track();
//       nav_queue_->push(nav_msg_);
//     } else if (channel_msg_.ChannelStatus == ChannelState::ACQUIRING) {
//       Acquire();
//     }
//     channel_queue_->push(channel_msg_);

//     // wait for all channels to finish
//     end_bar_->WaitFor(timeout_);

//     // wait for shm_ to be updated
//     start_bar_->WaitFor(timeout_);
//     // log_->error("GpsL1caChannel::run going??? still_running_ = {}", *still_running_);
//   }
//   // log_->warn("Made it out of GpsL1caChannel::run!");
// }

// *=== Acquire ===*
void GpsL1caChannel::Acquire() {
  try {
    samp_remaining_ = UnreadSampleCount();

    while (samp_remaining_ >= total_samp_) {
      // Perform acquisition
      Eigen::MatrixXd corr_map = PcpsSearch(
          shm_->segment(shm_read_ptr_, total_samp_),
          conf_.acquisition.num_coh_per,
          conf_.acquisition.num_noncoh_per,
          channel_msg_.Header.SVID,
          acq_setup_);
      // Eigen::MatrixXd corr_map = PcpsSearch(
      //     fft_plans_,
      //     shm_->segment(shm_read_ptr_, total_samp_),
      //     code_,
      //     conf_.acquisition.doppler_range,
      //     conf_.acquisition.doppler_step,
      //     conf_.rfsignal.samp_freq,
      //     satutils::GPS_CA_CODE_RATE<>,
      //     conf_.rfsignal.intmd_freq,
      //     conf_.acquisition.num_coh_per,
      //     conf_.acquisition.num_noncoh_per);
      int peak_idx[2];
      double metric;
      GlrtTest(corr_map, peak_idx, metric, shm_->segment(shm_read_ptr_, total_samp_));

      // did we acquire?
      if (metric > conf_.acquisition.threshold) {
        // --- SUCCESS ---
        channel_msg_.TrackingStatus |= TrackingFlags::ACQUIRED;
        channel_msg_.ChannelStatus = ChannelState::TRACKING;
        channel_msg_.Doppler = -conf_.acquisition.doppler_range +
                               static_cast<double>(peak_idx[0]) * conf_.acquisition.doppler_step;
        shm_read_ptr_ += (total_samp_ + peak_idx[1] - samp_per_ms_);
        shm_read_ptr_ %= shm_samp_chunk_size_;
        carr_doppler_ = navtools::TWO_PI<> * channel_msg_.Doppler;
        code_doppler_ = kappa_ * carr_doppler_;
        nco_carr_freq_ = intmd_freq_rad_ + carr_doppler_;
        nco_code_freq_ = satutils::GPS_CA_CODE_RATE<> + code_doppler_;

        // initialize tracking filter states
        filt_.Init(
            w0d_,
            w0p_,
            w0f_,
            kappa_,
            rem_carr_phase_,
            carr_doppler_,
            rem_code_phase_,
            cno_mag_,
            T_,
            intmd_freq_rad_,
            satutils::GPS_CA_CODE_RATE<>);

        // initialize tracking filter NCO
        double code_phase_step =
            (satutils::GPS_CA_CODE_RATE<> + code_doppler_) / conf_.rfsignal.samp_freq;
        total_samp_ = static_cast<uint64_t>(
            (static_cast<double>(PDI_ * satutils::GPS_CA_CODE_LENGTH) - rem_code_phase_) /
            code_phase_step);
        half_samp_ = total_samp_ / 2;
        samp_processed_ = 0;
        log_->info(
            "{}: GPS{} acquired! Doppler (Hz) = {:.0f}, Code Phase (samp) = {:d}, metric = {:.1f}",
            channel_id_,
            channel_msg_.Header.SVID,
            channel_msg_.Doppler,
            peak_idx[1],
            metric);
        // Track();
        return;
      }

      // --- FAILURE ---
      shm_read_ptr_ += total_samp_;
      shm_read_ptr_ %= shm_samp_chunk_size_;
      // channel_msg_.ChannelStatus = ChannelState::IDLE;
      log_->debug(
          "Acquisition for GPS{} on {} failed, metric = {:.1f}!",
          channel_msg_.Header.SVID,
          channel_id_,
          metric);
      // get new prn
      GetNewPrnFunc_(channel_msg_.Header.SVID);
      SetSatellite(channel_msg_.Header.SVID);

      // update samp_remaining_
      samp_remaining_ = UnreadSampleCount();
      acq_fails_++;
      if (acq_fails_ > conf_.acquisition.max_failed_attempts) {
        channel_msg_.ChannelStatus = ChannelState::IDLE;
        return;
      }
      // log_->error("acq_fails: {}, samp_remaining: {}", acq_fails_, samp_remaining_);
    }
  } catch (std::exception &e) {
    log_->error("gps-l1ca-channel.cpp Acquire failed! Error -> {}", e.what());
  }
}

// *=== Track ===*
void GpsL1caChannel::Track() {  // make sure samp_remaining_ > 0 to start
  try {
    samp_remaining_ = UnreadSampleCount();

    while (samp_remaining_ > 0) {
      if (samp_processed_ < total_samp_) {
        // --- INTEGRATE ---
        uint64_t samp_to_read = std::min(total_samp_ - samp_processed_, samp_remaining_);

        // accumulate samples of current code period
        AccumulateEPL(
            shm_->segment(shm_read_ptr_, samp_to_read),
            code_,
            rem_code_phase_,
            nco_code_freq_,
            rem_carr_phase_,
            nco_carr_freq_,
            carr_jitter_,
            conf_.rfsignal.samp_freq,
            samp_to_read,
            samp_processed_,
            half_samp_,
            tap_space_,
            E_,
            P1_,
            P2_,
            L_);
        P_ += (P1_ + P2_);

        // update shm_ ptr
        shm_read_ptr_ += samp_to_read;
        shm_read_ptr_ %= shm_samp_chunk_size_;
      } else {
        // --- DUMP ---
        if (!(channel_msg_.TrackingStatus & TrackingFlags::EPH_DECODED)) {
          // still parsing ephemeris
          if (!(channel_msg_.TrackingStatus & TrackingFlags::BIT_SYNC)) {
            // data bits not acquired - check for bit flip
            if (DataBitSync()) continue;
          } else {
            // synced to data bits - demodulate them into ephemeris
            Demodulate();
          }
        }

        // increment time of week estimate
        channel_msg_.ToW = channel_msg_.ToW + T_;
        nav_msg_.ToW = channel_msg_.ToW;

        // discriminators
        double t = static_cast<double>(total_samp_) / conf_.rfsignal.samp_freq;
        double half_t = 0.5 * t;
        double phase_err = PllCostas(P_);
        double freq_err = FllAtan2(P1_, P2_, half_t);
        double chip_err = DllNneml2(E_, L_);

        // lock detectors
        LockDetectors(
            code_lock_, carr_lock_, cno_mag_, nbd_mem_, nbp_mem_, pc_mem_, pn_mem_, P_old_, P_, t);

        // loop filter
        filt_.UpdateDynamicsParam(w0d_, w0p_, w0f_, kappa_, t);
        filt_.UpdateMeasurementsParam(cno_mag_, T_);
        filt_.Run(chip_err, phase_err, freq_err);
        rem_carr_phase_ = std::fmod(filt_.x_(0), navtools::TWO_PI<>);
        carr_doppler_ = filt_.x_(1);
        carr_jitter_ = filt_.x_(2);
        rem_code_phase_ = filt_.x_(3) - static_cast<double>(PDI_ * satutils::GPS_CA_CODE_LENGTH);
        code_doppler_ = filt_.x_(4) + kappa_ * (filt_.x_(1) + t * filt_.x_(2));
        nco_carr_freq_ = intmd_freq_rad_ + carr_doppler_;
        nco_code_freq_ = satutils::GPS_CA_CODE_RATE<> + code_doppler_;
        filt_.x_(0) = rem_carr_phase_;
        filt_.x_(3) = rem_code_phase_;
        // filt_.SetRemCarrierPhase(rem_carr_phase_);
        // filt_.SetRemCodePhase(rem_code_phase_);

        // update navigation message
        nav_msg_.CNo = cno_mag_;
        nav_msg_.Doppler = carr_doppler_ / navtools::TWO_PI<>;
        nav_msg_.CodePhase = rem_code_phase_;
        nav_msg_.CarrierPhase = rem_carr_phase_;

        // update tracking status
        if (code_lock_) {
          channel_msg_.TrackingStatus |= TrackingFlags::CODE_LOCK;
        } else {
          channel_msg_.TrackingStatus &= ~TrackingFlags::CODE_LOCK;
        }
        if (carr_lock_) {
          channel_msg_.TrackingStatus |= TrackingFlags::CARRIER_LOCK;
          if (track_mode_ < 1) {
            track_mode_ = 1;
            w0d_ = NaturalFrequency(conf_.tracking.dll_bw_standard, 2);
            w0p_ = NaturalFrequency(conf_.tracking.pll_bw_standard, 3);
            w0f_ = NaturalFrequency(conf_.tracking.fll_bw_standard, 2);
            channel_msg_.TrackingStatus &= ~TrackingFlags::FINE_LOCK;
          } else if (track_mode_ < 2) {
            track_mode_ = 2;
            w0d_ = NaturalFrequency(conf_.tracking.dll_bw_narrow, 2);
            w0p_ = NaturalFrequency(conf_.tracking.pll_bw_narrow, 3);
            w0f_ = NaturalFrequency(conf_.tracking.fll_bw_narrow, 2);
            channel_msg_.TrackingStatus |= TrackingFlags::FINE_LOCK;
          }
        } else {
          channel_msg_.TrackingStatus &= ~TrackingFlags::CARRIER_LOCK;
          if (track_mode_ > 1) {
            track_mode_ = 1;
            w0d_ = NaturalFrequency(conf_.tracking.dll_bw_standard, 2);
            w0p_ = NaturalFrequency(conf_.tracking.pll_bw_standard, 3);
            w0f_ = NaturalFrequency(conf_.tracking.fll_bw_standard, 2);
            channel_msg_.TrackingStatus &= ~TrackingFlags::FINE_LOCK;
          } else if (track_mode_ > 0) {
            track_mode_ = 0;
            w0d_ = NaturalFrequency(conf_.tracking.dll_bw_wide, 2);
            w0p_ = NaturalFrequency(conf_.tracking.pll_bw_wide, 3);
            w0f_ = NaturalFrequency(conf_.tracking.fll_bw_wide, 2);
            channel_msg_.TrackingStatus &= ~TrackingFlags::FINE_LOCK;
          }
        }

        // update channel message
        channel_msg_.CNo = (cno_mag_ > 0) ? (10.0 * std::log10(cno_mag_)) : 0.0;
        channel_msg_.Doppler = nav_msg_.Doppler;
        channel_msg_.CodePhase = rem_code_phase_;
        channel_msg_.CarrierPhase = rem_carr_phase_;
        channel_msg_.IE = E_.real();
        channel_msg_.IP = P_.real();
        channel_msg_.IL = L_.real();
        channel_msg_.QE = E_.imag();
        channel_msg_.QP = P_.imag();
        channel_msg_.QL = L_.imag();
        channel_msg_.IP1 = P1_.real();
        channel_msg_.IP2 = P2_.real();
        channel_msg_.QP1 = P1_.imag();
        channel_msg_.QP2 = P2_.imag();
        channel_msg_.DllDisc = chip_err;
        channel_msg_.PllDisc = phase_err;
        channel_msg_.FllDisc = freq_err;
        log_->debug(
            "{}: GPS{} - CNo = {:.1f}, Doppler = {:.1f}, IP = {:.1f}, QP = {:.1f}",
            channel_id_,
            channel_msg_.Header.SVID,
            channel_msg_.CNo,
            channel_msg_.Doppler,
            channel_msg_.IP,
            channel_msg_.QP);

        // prepare next nco period
        cnt_ += PDI_;
        double code_phase_step =
            (satutils::GPS_CA_CODE_RATE<> + code_doppler_) / conf_.rfsignal.samp_freq;
        total_samp_ = static_cast<uint64_t>(
            (static_cast<double>(PDI_ * satutils::GPS_CA_CODE_LENGTH) - rem_code_phase_) /
            code_phase_step);
        half_samp_ = total_samp_ / 2;
        samp_processed_ = 0;

        // reset correlators
        P_old_ = P_;
        E_ = std::complex<double>(0.0, 0.0);
        P_ = std::complex<double>(0.0, 0.0);
        L_ = std::complex<double>(0.0, 0.0);
        P1_ = std::complex<double>(0.0, 0.0);
        P2_ = std::complex<double>(0.0, 0.0);
      }

      // update samp_remaining_
      samp_remaining_ = UnreadSampleCount();
    }

    // save intermediate phases
    channel_msg_.CodePhase = rem_code_phase_;
    channel_msg_.CarrierPhase = rem_carr_phase_;
    nav_msg_.CodePhase = rem_code_phase_;
    nav_msg_.CarrierPhase = rem_carr_phase_;
  } catch (std::exception &e) {
    log_->error("gps-l1ca-channel.cpp Track failed! Error -> {}", e.what());
  }
}

// *=== DataBitSync ===*
bool GpsL1caChannel::DataBitSync() {
  try {
    // check for bit flip
    if ((P_.real() < 0.0) != (P_old_.real() < 0.0)) {
      bit_sync_hist_[cnt_ % 20]++;

      if (cnt_ > (int)conf_.tracking.min_converg_time_ms) {
        // test for data syncronization
        uint8_t sorted_arr[20];
        std::copy(bit_sync_hist_, bit_sync_hist_ + 20, sorted_arr);
        std::sort(sorted_arr, sorted_arr + 20);

        if (sorted_arr[19] > 4 * sorted_arr[18]) {
          log_->info(
              "{}: GPS{} data bit sync! cnt_ = {}", channel_id_, channel_msg_.Header.SVID, cnt_);
          channel_msg_.TrackingStatus |= TrackingFlags::BIT_SYNC;

          // narrow the filters
          tap_space_ = conf_.tracking.tap_epl_narrow;
          w0d_ = NaturalFrequency(conf_.tracking.dll_bw_narrow, 2);
          w0p_ = NaturalFrequency(conf_.tracking.pll_bw_narrow, 3);
          w0f_ = NaturalFrequency(conf_.tracking.fll_bw_narrow, 2);
          // TODO: implement checks for upgrading lock status
          channel_msg_.TrackingStatus |= TrackingFlags::FINE_LOCK;

          // immediately continue integrating with extended (20 ms) period
          T_ = 0.02;
          PDI_ = 20;
          double code_phase_step =
              (satutils::GPS_CA_CODE_RATE<> + code_doppler_) / conf_.rfsignal.samp_freq;
          total_samp_ = static_cast<uint64_t>(
              (static_cast<double>(PDI_ * satutils::GPS_CA_CODE_LENGTH) - rem_code_phase_) /
              code_phase_step);
          half_samp_ = total_samp_ / 2;
          P1_ += P2_;
          P2_ = std::complex<double>(0.0, 0.0);
          return true;
        }
      }
    }
    return false;
  } catch (std::exception &e) {
    log_->error("gps-l1ca-channel.cpp DataBitSync failed! Error -> {}", e.what());
    return false;
  }
}

// *=== Demodulate ===*
void GpsL1caChannel::Demodulate() {
  try {
    // Keep replacing bits while phase locked
    bool bit = P_.real() > 0.0;
    if (gps_lnav_.SetNextBit(bit)) {
      // reset sample count after subframe has been parsed
      channel_msg_.Week = gps_lnav_.GetWeekNumber();
      channel_msg_.ToW = gps_lnav_.GetTimeOfWeek() - 0.02;
      nav_msg_.Week = channel_msg_.Week;
      nav_msg_.ToW = channel_msg_.ToW;
      channel_msg_.TrackingStatus |= TrackingFlags::TOW_DECODED;
      channel_msg_.TrackingStatus |= TrackingFlags::SUBFRAME_SYNC;
    }

    // Check if complete ephemeris is parsed
    if (gps_lnav_.AreEphemeridesParsed()) {
      log_->info(
          "{}: GPS{} ephemeris subframes 1,2 and 3 parsed!", channel_id_, channel_msg_.Header.SVID);
      channel_msg_.TrackingStatus |= TrackingFlags::EPH_DECODED;
      // log_->debug(
      //     "{}: GPS{} {}", channel_id_, channel_msg_.Header.SVID, gps_lnav_.GetEphemerides());
    }
  } catch (std::exception &e) {
    log_->error("gps-l1ca-channel.cpp Demodulate failed! Error -> {}", e.what());
  }
}

}  // namespace sturdr