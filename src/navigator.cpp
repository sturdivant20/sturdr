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

#include <spdlog/sinks/basic_file_sink.h>

#include <cmath>
#include <navtools/frames.hpp>
#include <satutils/gnss-constants.hpp>
#include <sturdins/least-squares.hpp>
#include <sturdins/nav-clock.hpp>

#include "navtools/constants.hpp"
#include "satutils/ephemeris.hpp"

namespace sturdr {

// *=== Navigator ===*
Navigator::Navigator(
    Config &conf,
    std::shared_ptr<ConcurrentQueue<ChannelNavPacket>> &nav_queue,
    std::shared_ptr<ConcurrentQueue<ChannelEphemPacket>> &eph_queue,
    std::shared_ptr<bool> &running)
    : conf_{conf},
      receive_time_{0.0},
      lla_{Eigen::Vector3d::Zero()},
      nedv_{Eigen::Vector3d::Zero()},
      cb_{0.0},
      cd_{0.0},
      T_{1.0 / conf_.navigation.meas_freq},
      is_initialized_{false},
      nav_queue_{nav_queue},
      eph_queue_{eph_queue},
      update_{false},
      running_{running},
      thread_{std::thread(&Navigator::NavigationThread, this)},
      log_{spdlog::get("sturdr-console")},
      nav_log_{spdlog::basic_logger_st(
          "sturdr-navigation-log",
          conf_.general.out_folder + "/" + conf_.general.scenario + "/Navigation_Log.csv",
          true)},
      eph_log_{spdlog::basic_logger_st(
          "sturdr-ephemeris-log",
          conf_.general.out_folder + "/" + conf_.general.scenario + "/Ephemeris_Log.csv",
          true)} {
  // initialize navigation results logger
  nav_log_->set_pattern("%v");
  nav_log_->info(
      "Week,ToW [s],Lat [deg],Lon [deg],Alt [m],vN [m/s],vE [m/s],vD [m/s],b [m],bdot [m/s]");

  // initialize ephemeris logger
  eph_log_->set_pattern("%v");
  eph_log_->info(
      "SVID,iode,iodc,toe,toc,tgd,af2,af1,af0,e,sqrtA,deltan,m0,omega0,omega,omegaDot,i0,iDot,"
      "cuc,cus,cic,cis,crc,crs,ura,health");

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
  update_ = true;
  cv_.notify_one();
}

// *=== NavigationThread ===*
void Navigator::NavigationThread() {
  // initialize channel listeners
  std::thread nav_packet_t(&Navigator::ChannelNavPacketListener, this);
  std::thread eph_packet_t(&Navigator::ChannelEphemPacketListener, this);

  // do actual navigation work here
  while (*running_) {
    std::unique_lock<std::mutex> lock(mtx_);

    // TODO: figure out how to 'safely' wait for current queue items to process
    //  - meaning only the items from before the navigation update was requested
    while (nav_queue_->size() > 0);

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
      Eigen::MatrixXd sv_acc(3, num_sv);
      Eigen::MatrixXd sv_clk(3, num_sv);
      Eigen::VectorXd psr(num_sv);
      Eigen::VectorXd psrdot(num_sv);
      Eigen::VectorXd psr_var(num_sv);
      Eigen::VectorXd psrdot_var(num_sv);
      Eigen::VectorXd transmit_time(num_sv);
      Eigen::VectorXd tgd(num_sv);

      // get satellite position information
      for (uint8_t i = 0; i < num_sv; i++) {
        uint8_t it = good_sv[i];
        transmit_time(i) = channel_data_[it].ToW + channel_data_[it].CodePhaseSec;
        channel_data_[it].Sv.CalcNavStates<false>(
            sv_clk.col(i), sv_pos.col(i), sv_vel.col(i), sv_acc.col(i), transmit_time(i));
        tgd(i) = channel_data_[it].Sv.tgd;
        psrdot(i) = channel_data_[it].Psrdot;
        psr_var(i) = channel_data_[it].PsrVar;
        psrdot_var(i) = channel_data_[it].PsrdotVar;
      }

      // correct transmit time for satellite biases
      transmit_time = transmit_time.array() - sv_clk.row(0).transpose().array() + tgd.array();
      psrdot = psrdot.array() + navtools::LIGHT_SPEED<> * sv_clk.row(1).transpose().array();

      // run correct navigation update
      if (is_initialized_) {
        ScalarNavSolution(sv_pos, sv_vel, transmit_time, psrdot, psr_var, psrdot_var);
      } else {
        InitNavSolution(sv_pos, sv_vel, transmit_time, psrdot, psr_var, psrdot_var);
      }

      // log results
      log_->info("\tLLA:\t{:.8f}, {:.8f}, {:.1f}", lla_(0), lla_(1), lla_(2));
      log_->info("\tNEDV:\t{:.3f}, {:.3f}, {:.3f}", nedv_(0), nedv_(1), nedv_(2));
      log_->info("\tCLK:\t{:.1f}, {:.3f}", cb_, cd_);
      nav_log_->info(
          "{}, {}, {}, {}, {}, {}, {}, {}",
          lla_(0),
          lla_(1),
          lla_(2),
          nedv_(0),
          nedv_(1),
          nedv_(2),
          cb_,
          cd_);
    }

    // wait for navigation update
    update_ = false;
    cv_.wait(lock, [this] { return update_ || !*running_; });
  }

  nav_packet_t.join();
  eph_packet_t.join();
}

// *=== ChannelNavPacketListener ===*
void Navigator::ChannelNavPacketListener() {
  ChannelNavPacket packet;
  while (nav_queue_->pop(packet) && *running_) {
    std::unique_lock<std::mutex> lock(mtx_);
    if (channel_data_.find(packet.Header.ChannelNum) != channel_data_.end()) {
      // update map data
      channel_data_[packet.Header.ChannelNum].Week = packet.Week;
      channel_data_[packet.Header.ChannelNum].ToW = packet.ToW;
      channel_data_[packet.Header.ChannelNum].CNo = packet.CNo;
      channel_data_[packet.Header.ChannelNum].Psrdot = packet.Psrdot;
      channel_data_[packet.Header.ChannelNum].CodePhaseSec = packet.CodePhaseSec;
      channel_data_[packet.Header.ChannelNum].CarrierPhase = packet.CarrierPhase;
      channel_data_[packet.Header.ChannelNum].DllDisc = packet.DllDisc;
      channel_data_[packet.Header.ChannelNum].PllDisc = packet.PllDisc;
      channel_data_[packet.Header.ChannelNum].FllDisc = packet.FllDisc;
      channel_data_[packet.Header.ChannelNum].PsrVar = packet.PsrVar;
      channel_data_[packet.Header.ChannelNum].PsrdotVar = packet.PsrdotVar;
      channel_data_[packet.Header.ChannelNum].HasData = true;
    } else {
      // add new item to map
      channel_data_.insert(
          {packet.Header.ChannelNum,
           ChannelNavData{
               satutils::KeplerEphem<double>(),
               packet.Week,
               packet.ToW,
               packet.CNo,
               packet.Psrdot,
               packet.CodePhaseSec,
               packet.CarrierPhase,
               packet.DllDisc,
               packet.PllDisc,
               packet.FllDisc,
               packet.PsrVar,
               packet.PsrdotVar,
               false,
               true}});
    }
  }
}

// *=== ChannelEphemPacketListener ===*
void Navigator::ChannelEphemPacketListener() {
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
               true,
               false}});
    }

    // log ephemeris
    eph_log_->info("GPS{},{}", packet.Header.SVID, packet.Eph);
  }
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
  receive_time_ = transmit_time.maxCoeff() + conf_.navigation.nominal_transit_time;
  Eigen::VectorXd psr = (receive_time_ - transmit_time.array()) * navtools::LIGHT_SPEED<>;

  // std::cout << "#: transmit time | psr(var) | psrdot(var)\n" << std::setprecision(11);
  // for (uint8_t i = 0; i < num_sv; i++) {
  //   std::cout << (int)good_sv[i] << ": " << transmit_time(i) << " | " << psr(i) << " ("
  //             << psr_var(i) << ") | " << psrdot(i) << " (" << psrdot_var(i) << ")\n";
  // }
  // std::cout << "\n#: sv_pos | sv_vel\n" << std::setprecision(10);
  // for (uint8_t i = 0; i < num_sv; i++) {
  //   std::cout << (int)good_sv[i] << ": " << sv_pos(0, i) << ", " << sv_pos(1, i) << ", "
  //             << sv_pos(2, i) << " | " << sv_vel(0, i) << ", " << sv_vel(1, i) << ", "
  //             << sv_vel(2, i) << "\n";
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
  receive_time_ -= cb_ / navtools::LIGHT_SPEED<>;
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
    const Eigen::Ref<const Eigen::VectorXd> &psrdot_var) {
  // propagate pseudoranges and rates
  receive_time_ += T_;
  Eigen::VectorXd psr = (receive_time_ - transmit_time.array()) * navtools::LIGHT_SPEED<>;

  // propagate navigation filter
  kf_.Propagate(T_);
  kf_.GnssUpdate(sv_pos, sv_vel, psr, psrdot, psr_var, psrdot_var);
  lla_ << navtools::RAD2DEG<> * kf_.phi_, navtools::RAD2DEG<> * kf_.lam_, kf_.h_;
  nedv_ << kf_.vn_, kf_.ve_, kf_.vd_;
  cb_ = kf_.cb_;
  cd_ = kf_.cd_;
}

// *=== VectorNavSolution ===*
void Navigator::VectorNavSolution() {
}

}  // namespace sturdr