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
#include <iomanip>
#include <navtools/frames.hpp>
#include <satutils/gnss-constants.hpp>
#include <sturdins/least-squares.hpp>

#include "navtools/constants.hpp"
#include "satutils/ephemeris.hpp"
#include "sturdr/discriminator.hpp"

namespace sturdr {

inline constexpr double BETA2 = navtools::LIGHT_SPEED<> * navtools::LIGHT_SPEED<> /
                                satutils::GPS_CA_CODE_RATE<> / satutils::GPS_CA_CODE_RATE<>;
inline constexpr double LAMBDA2 = navtools::LIGHT_SPEED<> * navtools::LIGHT_SPEED<> /
                                  satutils::GPS_L1_FREQUENCY<> / satutils::GPS_L1_FREQUENCY<>;

// *=== Navigator ===*
Navigator::Navigator(
    Config conf,
    std::shared_ptr<ConcurrentQueue<ChannelNavPacket>> nav_queue,
    std::shared_ptr<ConcurrentQueue<ChannelEphemPacket>> eph_queue,
    std::shared_ptr<bool> running)
    : conf_{conf},
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
}

// *=== ~Navigator ===*
Navigator::~Navigator() {
}

// *=== Execute ===*
void Navigator::Execute() {
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
    cv_.wait(lock, [this] { return update_ || !*running_; });

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
      std::cout << "running navigation update\n";
      // run navigation update
      Eigen::VectorXd x{Eigen::Vector<double, 8>::Zero()};
      Eigen::MatrixXd P{Eigen::Matrix<double, 8, 8>::Zero()};
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

      // get satellite position
      // std::cout << "Transmit time: " << std::setprecision(17);
      // std::cout << "Doppler: " << std::setprecision(17) << '\n';
      // std::cout << "CNo: " << std::setprecision(17) << '\n';
      // std::cout << "sv_pos: " << std::setprecision(17) << '\n';
      // std::cout << "sv_vel: " << std::setprecision(17) << '\n';
      for (uint8_t i = 0; i < num_sv; i++) {
        uint8_t it = good_sv[i];
        transmit_time(i) =
            channel_data_[it].ToW + channel_data_[it].CodePhase / satutils::GPS_CA_CODE_RATE<>;
        channel_data_[it].Sv.CalcNavStates<false>(
            sv_clk.col(i), sv_pos.col(i), sv_vel.col(i), sv_acc.col(i), transmit_time(i));
        tgd(i) = channel_data_[it].Sv.tgd;
        psrdot(i) = channel_data_[it].Doppler;
        psr_var(i) = BETA2 * DllVariance(channel_data_[it].CNo, 0.02);
        psrdot_var(i) = LAMBDA2 * FllVariance(channel_data_[it].CNo, 0.02);
        // std::cout << (int)it << ": " << transmit_time(i) << " (" << psr_var(i) << ")\n";
        // std::cout << (int)it << ": " << psrdot(i) << " (" << psrdot_var(i) << ")\n";
        // std::cout << (int)it << ": " << channel_data_[it].CNo << "("
        //           << 10.0 * std::log10(channel_data_[it].CNo) << ")\n";
        // std::cout << (int)it << ": " << sv_pos(0, i) << ", " << sv_pos(1, i) << ", " << sv_pos(2,
        // i)
        // << "\n";
        // std::cout << (int)it << ": " << sv_vel(0, i) << ", " << sv_vel(1, i) << ", " << sv_vel(2,
        // i)
        //           << "\n";
      }
      // std::cout << "\n";

      double receive_time = transmit_time.maxCoeff() + conf_.navigation.nominal_transit_time;
      psr =
          (receive_time - transmit_time.array() + sv_clk.row(0).transpose().array() + tgd.array()) *
          navtools::LIGHT_SPEED<>;
      psrdot = navtools::LIGHT_SPEED<> *
               (-psrdot.array() / satutils::GPS_L1_FREQUENCY<> + sv_clk.row(1).transpose().array());

      std::cout << "#: transmit time | psr | psrdot\n" << std::setprecision(11);
      for (uint8_t i = 0; i < num_sv; i++) {
        std::cout << (int)good_sv[i] << ": " << transmit_time(i) << " | " << psr(i) << " ("
                  << psr_var(i) << ") | " << psrdot(i) << " (" << psrdot_var(i) << ")\n";
      }
      std::cout << "\n#: sv_pos | sv_vel\n" << std::setprecision(10);
      for (uint8_t i = 0; i < num_sv; i++) {
        std::cout << (int)good_sv[i] << ": " << sv_pos(0, i) << ", " << sv_pos(1, i) << ", "
                  << sv_pos(2, i) << " | " << sv_vel(0, i) << ", " << sv_vel(1, i) << ", "
                  << sv_vel(2, i) << "\n";
      }

      // least squares
      sturdins::GaussNewton(x, P, sv_pos, sv_vel, psr, psrdot, psr_var, psrdot_var);
      // Eigen::Vector3d xyz = x.segment(0, 3), lla;
      navtools::ecef2lla<double>(lla_, x.segment(0, 3));
      log_->info(
          "XYZ: {}, {}, {}, {}, {}, {}, {}, {}", x(0), x(1), x(2), x(3), x(4), x(5), x(6), x(7));
      log_->info(
          "LLA: {}, {}, {}", navtools::RAD2DEG<> * lla_(0), navtools::RAD2DEG<> * lla_(1), lla_(2));
    }

    update_ = false;
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
      channel_data_[packet.Header.ChannelNum].Doppler = packet.Doppler;
      channel_data_[packet.Header.ChannelNum].CodePhase = packet.CodePhase;
      channel_data_[packet.Header.ChannelNum].CarrierPhase = packet.CarrierPhase;
      channel_data_[packet.Header.ChannelNum].DllDisc = packet.DllDisc;
      channel_data_[packet.Header.ChannelNum].PllDisc = packet.PllDisc;
      channel_data_[packet.Header.ChannelNum].FllDisc = packet.FllDisc;
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
               packet.Doppler,
               packet.CodePhase,
               packet.CarrierPhase,
               packet.DllDisc,
               packet.PllDisc,
               packet.FllDisc,
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

}  // namespace sturdr