/**
 * *navigator.hpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/navigator.hpp
 * @brief   Handles navigation from the tracking states in each channel
 * @date    January 2025
 * @ref     1. "Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems", 2nd
 *              Edition, 2013 - Groves
 *          2. "Global Positioning System: Signals, Measurements, and Performance", 2nd Edition,
 *              2006 - Misra & Enge
 *          3. "A Software-Defined GPS and Galileo Receiver: A Single-Frequency Approach", 2007
 *            - Borre, Akos, Bertelsen, Rinder, Jensen
 * =======  ========================================================================================
 */

// TODO: add covariance (diagonal) to navigation log

#include "sturdr/navigator.hpp"

#include <fmt/ranges.h>
#include <spdlog/async.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/sinks/basic_file_sink.h>

#include <cmath>
#include <cstdio>
#include <memory>
#include <mutex>
#include <navtools/frames.hpp>
#include <satutils/gnss-constants.hpp>
#include <sturdins/least-squares.hpp>
#include <sturdins/nav-clock.hpp>
#include <vector>

#include "navtools/constants.hpp"
#include "satutils/ephemeris.hpp"
#include "sturdr/structs-enums.hpp"
#include "sturdr/vector-tracking.hpp"

namespace sturdr {

// *=== Navigator ===*
Navigator::Navigator(
    Config &conf,
    std::shared_ptr<ConcurrentQueue<ChannelNavPacket>> &nav_queue,
    std::shared_ptr<ConcurrentQueue<ChannelEphemPacket>> &eph_queue,
    std::shared_ptr<bool> &running)
    : conf_{conf},
      file_ptr_{0},
      file_size_{
          conf_.general.ms_chunk_size * static_cast<uint64_t>(conf_.rfsignal.samp_freq) / 1000},
      receive_time_{0.0},
      lla_{Eigen::Vector3d::Zero()},
      nedv_{Eigen::Vector3d::Zero()},
      cb_{0.0},
      cd_{0.0},
      is_initialized_{false},
      is_vector_{false},
      nav_queue_{nav_queue},
      eph_queue_{eph_queue},
      update_{false},
      running_{running},
      thread_{std::thread(&Navigator::NavigationThread, this)},
      log_{spdlog::get("sturdr-console")} {
  // set known parameters for kalman filter
  sturdins::NavigationClock clk = sturdins::GetNavClock(conf_.navigation.clock_model);
  kf_.SetClockSpec(clk.h0, clk.h1, clk.h2);
  kf_.SetProcessNoise(conf_.navigation.process_std * conf_.navigation.process_std);

  log_->info("SturDR Navigator initialized");
}

// *=== ~Navigator ===*
Navigator::~Navigator() {
  if (thread_.joinable()) {
    thread_.join();
  }
}

// *=== Execute ===*
void Navigator::Notify() {
  {
    std::unique_lock<std::mutex> event_lock(event_mtx_);
    update_ = true;
  }
  cv_.notify_one();
}

// *=== NavigationThread ===*
void Navigator::NavigationThread() {
  // initialize channel listeners
  std::thread nav_packet_t(&Navigator::ChannelNavPacketListener, this);
  std::thread eph_packet_t(&Navigator::ChannelEphemPacketListener, this);

  // initialize navigation logger
  nav_log_ = spdlog::basic_logger_mt<spdlog::async_factory>(
      "sturdr-navigation-log",
      conf_.general.out_folder + "/" + conf_.general.scenario + "/Navigation_Log.csv",
      true);
  nav_log_->set_pattern("%v");
  nav_log_->info(
      "Week,ToW [s],Lat [deg],Lon [deg],Alt [m],vN [m/s],vE [m/s],vD [m/s],b [m],bdot [m/s]");

  // do actual navigation work here
  while (*running_) {
    // TODO: figure out how to 'safely' wait for current queue items to process
    //  - meaning only the items from before the navigation update was requested
    if (!is_vector_) {
      while (nav_queue_->size() > 0) continue;
      {
        std::unique_lock<std::mutex> lock(mtx_);
        if (NavigationUpdate()) {
          // log results
          log_->debug("\tLLA:\t{:.8f}, {:.8f}, {:.2f}", lla_(0), lla_(1), lla_(2));
          // log_->info("\tNEDV:\t{:.3f}, {:.3f}, {:.3f}", nedv_(0), nedv_(1), nedv_(2));
          // log_->info("\tCLK:\t{:.2f}, {:.3f}", cb_, cd_);
          nav_log_->info(
              "{},{:.17f},{:.15f},{:.15f},{:.11f},{:.15f},{:.15f},{:.15f},{:.11f},{:.15f}",
              week_,
              receive_time_,
              lla_(0),
              lla_(1),
              lla_(2),
              nedv_(0),
              nedv_(1),
              nedv_(2),
              cb_,
              cd_);
        }
      }
    }

    // wait for navigation update
    update_ = false;
    std::unique_lock<std::mutex> event_lock(event_mtx_);
    cv_.wait(event_lock, [this] { return update_ || !*running_; });
  }

  // kill listener threads
  nav_packet_t.join();
  eph_packet_t.join();

  // make sure no channels get stuck waiting for vector updates
  for (ChannelSyncData &it : channel_sync_) {
    *it.update_complete = true;
    it.cv->notify_all();
  }
}

// *=== ChannelNavPacketListener ===*
void Navigator::ChannelNavPacketListener() {
  ChannelNavPacket packet;
  while (nav_queue_->pop(packet) && *running_) {
    std::unique_lock<std::mutex> lock(mtx_);
    std::unique_lock<std::mutex> channel_lock(*packet.mtx);
    if (channel_data_.find(packet.Header.ChannelNum) != channel_data_.end()) {
      // update map data
      channel_data_[packet.Header.ChannelNum].FilePtr = packet.FilePtr;
      channel_data_[packet.Header.ChannelNum].Week = packet.Week;
      channel_data_[packet.Header.ChannelNum].ToW = packet.ToW;
      channel_data_[packet.Header.ChannelNum].Doppler = packet.Doppler;
      channel_data_[packet.Header.ChannelNum].CodePhase = packet.CodePhase;
      channel_data_[packet.Header.ChannelNum].CarrierPhase = packet.CarrierPhase;
      channel_data_[packet.Header.ChannelNum].DllDisc = packet.DllDisc;
      channel_data_[packet.Header.ChannelNum].PllDisc = packet.PllDisc;
      channel_data_[packet.Header.ChannelNum].FllDisc = packet.FllDisc;
      channel_data_[packet.Header.ChannelNum].PsrVar = packet.PsrVar;
      channel_data_[packet.Header.ChannelNum].PsrdotVar = packet.PsrdotVar;
      channel_data_[packet.Header.ChannelNum].Beta = packet.Beta;
      channel_data_[packet.Header.ChannelNum].Lambda = packet.Lambda;
      channel_data_[packet.Header.ChannelNum].ChipRate = packet.ChipRate;
      channel_data_[packet.Header.ChannelNum].CarrierFreq = packet.CarrierFreq;
      channel_data_[packet.Header.ChannelNum].HasData = true;
      channel_data_[packet.Header.ChannelNum].VTCodeRate = packet.VTCodeRate;
      channel_data_[packet.Header.ChannelNum].VTCarrierFreq = packet.VTCarrierFreq;
      channel_data_[packet.Header.ChannelNum].ReadyForVT = false;
      // channel_sync_[packet.Header.ChannelNum].cv = packet.cv;
      // channel_sync_[packet.Header.ChannelNum].update_complete = packet.update_complete;
      // channel_sync_[packet.Header.ChannelNum].is_vector = packet.is_vector;
    } else {
      // add new item to map
      channel_data_.insert(
          {packet.Header.ChannelNum,
           ChannelNavData{
               packet.FilePtr,
               satutils::KeplerEphem<double>(),
               packet.Week,
               packet.ToW,
               packet.Doppler,
               packet.CodePhase,
               packet.CarrierPhase,
               packet.DllDisc,
               packet.PllDisc,
               packet.FllDisc,
               packet.PsrVar,
               packet.PsrdotVar,
               packet.Beta,
               packet.Lambda,
               packet.ChipRate,
               packet.CarrierFreq,
               false,
               true,
               false,
               packet.VTCodeRate,
               packet.VTCarrierFreq}});
      channel_sync_.push_back({packet.cv, packet.update_complete, packet.is_vector, false});
    }

    // run vector tracking updates
    if (is_vector_) {
      // log_->warn(
      //     "Channel {} - GPS{:02d} requesting vector update (fd: {:.3f}, fd,ca: {:.3f}, ptr: {},"
      //     "size: {}, Ready4VT: {})",
      //     packet.Header.ChannelNum,
      //     packet.Header.SVID,
      //     packet.Doppler / navtools::TWO_PI<>,
      //     *packet.VTCodeRate - satutils::GPS_CA_CODE_RATE<>,
      //     packet.FilePtr,
      //     file_size_,
      //     channel_data_[packet.Header.ChannelNum].ReadyForVT);
      channel_data_[packet.Header.ChannelNum].ReadyForVT = true;
      // std::cout << "Ready4VT: [ ";
      // for (auto &it : channel_data_) {
      //   std::cout << it.second.ReadyForVT << " ";
      // }
      // std::cout << "]\n";
      VectorNavSolution();

    } else {
      // notify channel to continue
      *packet.update_complete = true;
      packet.cv->notify_one();
    }
  }
}

// *=== ChannelEphemPacketListener ===*
void Navigator::ChannelEphemPacketListener() {
  // initialize ephemeris logger
  eph_log_ = spdlog::basic_logger_st(
      "sturdr-ephemeris-log",
      conf_.general.out_folder + "/" + conf_.general.scenario + "/Ephemeris_Log.csv",
      true);
  eph_log_->set_pattern("%v");
  eph_log_->info(
      "SVID,iode,iodc,toe,toc,tgd,af2,af1,af0,e,sqrtA,deltan,m0,omega0,omega,omegaDot,i0,iDot,"
      "cuc,cus,cic,cis,crc,crs,ura,health");

  ChannelEphemPacket packet;
  while (eph_queue_->pop(packet) && *running_) {
    std::unique_lock<std::mutex> lock(mtx_);
    if (channel_data_.find(packet.Header.ChannelNum) != channel_data_.end()) {
      // update map data
      channel_data_[packet.Header.ChannelNum].Sv.SetEphem(packet.Eph);
      channel_data_[packet.Header.ChannelNum].HasEphem = true;
    } else {
      // add new item to map
      channel_data_.insert(
          {packet.Header.ChannelNum,
           ChannelNavData{
               0,
               satutils::KeplerEphem<double>(packet.Eph),
               0,
               0.0,
               0.0,
               0.0,
               0.0,
               0.0,
               0.0,
               0.0,
               0.0,
               0.0,
               0.0,
               0.0,
               0.0,
               0.0,
               true,
               false,
               false,
               std::make_shared<double>(0.0),
               std::make_shared<double>(0.0)}});
    }

    // log ephemeris
    eph_log_->info("GPS{},{}", packet.Header.SVID, packet.Eph);
  }
}

// *=== NavigationUpdate ===
bool Navigator::NavigationUpdate() {
  // make sure enough satellites have provided ephemeris
  std::vector<uint8_t> good_sv;
  uint8_t num_sv = 0;
  for (const std::pair<const uint8_t, sturdr::ChannelNavData> &it : channel_data_) {
    if (it.second.HasData && it.second.HasEphem) {
      good_sv.push_back(it.first);
      num_sv++;
    }
  }

  if (num_sv > 3) {
    // perform navigation update
    Eigen::MatrixXd sv_pos(3, num_sv);
    Eigen::MatrixXd sv_vel(3, num_sv);
    Eigen::Vector3d sv_acc;
    Eigen::Vector3d sv_clk;
    Eigen::VectorXd psr(num_sv);
    Eigen::VectorXd psrdot(num_sv);
    Eigen::VectorXd psr_var(num_sv);
    Eigen::VectorXd psrdot_var(num_sv);
    Eigen::VectorXd transmit_time(num_sv);
    Eigen::VectorXd file_ptrs(num_sv);

    // get satellite position information
    for (uint8_t i = 0; i < num_sv; i++) {
      uint8_t it = good_sv[i];
      file_ptrs(i) = channel_data_[it].FilePtr;

      transmit_time(i) = channel_data_[it].ToW +
                         channel_data_[it].CodePhase / channel_data_[it].ChipRate +
                         channel_data_[it].Sv.tgd;  //! FOR GPS L1CA
      channel_data_[it].Sv.CalcNavStates<false>(
          sv_clk, sv_pos.col(i), sv_vel.col(i), sv_acc, transmit_time(i));

      // correct transmit times/drifts for satellite biases (FOR GPS L1CA)
      transmit_time(i) -= sv_clk(0);
      psrdot(i) = -channel_data_[it].Lambda * channel_data_[it].Doppler +
                  navtools::LIGHT_SPEED<> * sv_clk(1);

      // reported variances from tracking loops
      psr_var(i) = channel_data_[it].PsrVar;
      psrdot_var(i) = channel_data_[it].PsrdotVar;
    }

    // run correct navigation update
    if (is_initialized_) {
      uint64_t d_samp = GetDeltaSamples(file_ptrs.minCoeff());
      // log_->warn(
      //     "d_samp = {} ({})", d_samp, static_cast<double>(d_samp) / conf_.rfsignal.samp_freq);
      ScalarNavSolution(sv_pos, sv_vel, transmit_time, psrdot, psr_var, psrdot_var, d_samp);
      UpdateFilePtr(d_samp);
    } else {
      InitNavSolution(sv_pos, sv_vel, transmit_time, psrdot, psr_var, psrdot_var);
      week_ = channel_data_[good_sv[0]].Week;
      file_ptr_ = file_ptrs.minCoeff();

      if (conf_.navigation.do_vt) {
        for (ChannelSyncData &it : channel_sync_) {
          *it.is_vector = true;
        }
        is_vector_ = true;
        log_->info("Channels will now begin vector tracking!");
      }
    }
    return true;
  }
  return false;
}

// *=== InitNavSolution ===*
void Navigator::InitNavSolution(
    const Eigen::Ref<const Eigen::MatrixXd> &sv_pos,
    const Eigen::Ref<const Eigen::MatrixXd> &sv_vel,
    const Eigen::Ref<const Eigen::VectorXd> &transmit_time,
    const Eigen::Ref<const Eigen::VectorXd> &psrdot,
    const Eigen::Ref<const Eigen::VectorXd> &psr_var,
    const Eigen::Ref<const Eigen::VectorXd> &psrdot_var) {
  // initial receive time guess
  receive_time_ = transmit_time.minCoeff() + conf_.navigation.nominal_transit_time;
  Eigen::VectorXd psr = (receive_time_ - transmit_time.array()) * navtools::LIGHT_SPEED<>;

  // std::cout << "transmit time: [ " << std::setprecision(17);
  // for (uint8_t i = 0; i < transmit_time.size(); i++) {
  //   std::cout << transmit_time(i) << ", ";
  // }
  // std::cout << "]\n";
  // std::cout << "#: transmit time | psr(var) | psrdot(var)\n";
  // for (uint8_t i = 0; i < transmit_time.size(); i++) {
  //   std::cout << (int)i << ": " << std::setprecision(17) << transmit_time(i)
  //             << std::setprecision(11) << " | " << psr(i) << " (" << psr_var(i) << ") | "
  //             << psrdot(i) << " (" << psrdot_var(i) << ")\n";
  // }
  // std::cout << "\n#: sv_pos | sv_vel\n" << std::setprecision(10);
  // for (uint8_t i = 0; i < transmit_time.size(); i++) {
  //   std::cout << (int)i << ": " << sv_pos(0, i) << ", " << sv_pos(1, i) << ", " << sv_pos(2, i)
  //             << " | " << sv_vel(0, i) << ", " << sv_vel(1, i) << ", " << sv_vel(2, i) << "\n";
  // }

  // least squares
  Eigen::VectorXd x{Eigen::Vector<double, 8>::Zero()};
  Eigen::MatrixXd P{Eigen::Matrix<double, 8, 8>::Zero()};
  sturdins::GaussNewton(x, P, sv_pos, sv_vel, psr, psrdot, psr_var, psrdot_var);
  navtools::ecef2lla<double>(lla_, x.segment(0, 3));
  navtools::ecef2nedv<double>(nedv_, x.segment(3, 3), lla_);
  cb_ = x(6);
  cd_ = x(7);

  // correct clock bias in receive time
  receive_time_ -= (cb_ / navtools::LIGHT_SPEED<>);
  cb_ = 0.0;

  // initialize kalman filter
  kf_.SetPosition(lla_(0), lla_(1), lla_(2));
  kf_.SetVelocity(nedv_(0), nedv_(1), nedv_(2));
  kf_.SetClock(cb_, cd_);
  lla_(0) *= navtools::RAD2DEG<>;
  lla_(1) *= navtools::RAD2DEG<>;
  is_initialized_ = true;
}

// *=== ScalarNavSolution ===*
void Navigator::ScalarNavSolution(
    const Eigen::Ref<const Eigen::MatrixXd> &sv_pos,
    const Eigen::Ref<const Eigen::MatrixXd> &sv_vel,
    const Eigen::Ref<const Eigen::VectorXd> &transmit_time,
    const Eigen::Ref<const Eigen::VectorXd> &psrdot,
    const Eigen::Ref<const Eigen::VectorXd> &psr_var,
    const Eigen::Ref<const Eigen::VectorXd> &psrdot_var,
    const uint64_t &d_samp) {
  // propagate pseudoranges and rates
  double dt = (static_cast<double>(d_samp) / conf_.rfsignal.samp_freq);
  receive_time_ += dt;
  Eigen::VectorXd psr = (receive_time_ - transmit_time.array()) * navtools::LIGHT_SPEED<>;

  // std::cout << "transmit time: [ " << std::setprecision(17);
  // for (uint8_t i = 0; i < transmit_time.size(); i++) {
  //   std::cout << transmit_time(i) << ", ";
  // }
  // std::cout << "]\n";
  // std::cout << "#: transmit time | psr(var) | psrdot(var)\n";
  // for (uint8_t i = 0; i < transmit_time.size(); i++) {
  //   std::cout << (int)i << ": " << std::setprecision(17) << transmit_time(i)
  //             << std::setprecision(11) << " | " << psr(i) << " (" << psr_var(i) << ") | "
  //             << psrdot(i) << " (" << psrdot_var(i) << ")\n";
  // }
  // std::cout << "\n#: sv_pos | sv_vel\n" << std::setprecision(10);
  // for (uint8_t i = 0; i < transmit_time.size(); i++) {
  //   std::cout << (int)i << ": " << sv_pos(0, i) << ", " << sv_pos(1, i) << ", " << sv_pos(2, i)
  //             << " | " << sv_vel(0, i) << ", " << sv_vel(1, i) << ", " << sv_vel(2, i) << "\n";
  // }

  // propagate navigation filter
  kf_.Propagate(dt);
  kf_.GnssUpdate(sv_pos, sv_vel, psr, psrdot, psr_var, psrdot_var);
  lla_ << navtools::RAD2DEG<> * kf_.phi_, navtools::RAD2DEG<> * kf_.lam_, kf_.h_;
  nedv_ << kf_.vn_, kf_.ve_, kf_.vd_;
  cb_ = kf_.cb_;
  cd_ = kf_.cd_;
}

// *=== VectorNavSolution ===*
void Navigator::VectorNavSolution() {
  // 1. Check and make sure all channels have requested an update
  // log_->info("VectorNavSolution called!");
  std::vector<std::pair<uint64_t, uint8_t>> sample_ptrs;
  for (const std::pair<const uint8_t, sturdr::ChannelNavData> &it : channel_data_) {
    if (!it.second.ReadyForVT) return;
    sample_ptrs.push_back({(it.second.FilePtr + file_ptr_) % file_size_, it.first});
    // sample_ptrs.push_back({it.second.FilePtr, it.first});
  }

  // log_->error("Vector update starting!");
  double intmd_freq_rad = navtools::TWO_PI<> * conf_.rfsignal.intmd_freq;
  double T = 0.02;

  // 2. Order updates based on the number of samples they wish to progress
  std::sort(sample_ptrs.begin(), sample_ptrs.end());
  std::vector<int> tmp;
  for (auto &it : channel_data_) {
    tmp.push_back(static_cast<int>(it.second.FilePtr));
  }
  // log_->error("sample_ptrs: {}, current_file_ptr: {}", tmp, file_ptr_);
  // std::cout << "sample_ptrs: [ ";
  // std::copy(tmp.begin(), tmp.end(), std::ostream_iterator<int>(std::cout, " "));
  // std::cout << "], file_ptr: " << file_ptr_ << "\n";

  uint64_t d_samp;
  for (std::pair<uint64_t, uint8_t> &p : sample_ptrs) {
    // 3. make sure samples are 'delta' from the last update
    d_samp = GetDeltaSamples(channel_data_[p.second].FilePtr);

    // 4. Run all vector updates and log to file
    if (d_samp > 0) {
      RunVDFllUpdate(
          d_samp,
          conf_.rfsignal.samp_freq,
          intmd_freq_rad,
          channel_data_[p.second],
          receive_time_,
          T,
          kf_);

      // 5. Update file pointer
      UpdateFilePtr(d_samp);

      // 6. Log solution
      lla_ << navtools::RAD2DEG<> * kf_.phi_, navtools::RAD2DEG<> * kf_.lam_, kf_.h_;
      nedv_ << kf_.vn_, kf_.ve_, kf_.vd_;
      cb_ = kf_.cb_;
      cd_ = kf_.cd_;
      log_->debug("\tLLA:\t{:.8f}, {:.8f}, {:.2f}", lla_(0), lla_(1), lla_(2));
      // log_->info("\tNEDV:\t{:.3f}, {:.3f}, {:.3f}", nedv_(0), nedv_(1), nedv_(2));
      // log_->info("\tCLK:\t{:.2f}, {:.3f}", cb_, cd_);
      nav_log_->info(
          "{},{:.17f},{:.15f},{:.15f},{:.11f},{:.15f},{:.15f},{:.15f},{:.11f},{:.15f}",
          week_,
          receive_time_,
          lla_(0),
          lla_(1),
          lla_(2),
          nedv_(0),
          nedv_(1),
          nedv_(2),
          cb_,
          cd_);
    }
  }

  // 6. Tell channels to continue working hard
  for (auto &it : channel_data_) {
    it.second.ReadyForVT = false;
  }
  for (ChannelSyncData &it : channel_sync_) {
    *it.update_complete = true;
    it.cv->notify_one();
  }
}

// *=== UpdateFilePtr ===*
void Navigator::UpdateFilePtr(const uint64_t &d_samp) {
  file_ptr_ += d_samp;
  file_ptr_ %= file_size_;
}

// *=== GetDeltaSamples ===*
uint64_t Navigator::GetDeltaSamples(const uint64_t &new_file_ptr) {
  if (file_ptr_ <= new_file_ptr) {
    return new_file_ptr - file_ptr_;
  } else {
    return file_size_ - file_ptr_ + new_file_ptr;
  }
}

}  // namespace sturdr