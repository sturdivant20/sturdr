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

#include "sturdr/navigator.hpp"

#include <chrono>
#include <cmath>
#include <fastdds/dds/core/policy/QosPolicies.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/dds/publisher/qos/PublisherQos.hpp>
#include <iostream>
#include <mutex>
#include <navtools/attitude.hpp>
#include <navtools/frames.hpp>
#include <navtools/math.hpp>
#include <sturdds/ChannelMessagePubSubTypes.hpp>
#include <sturdds/NavMessagePubSubTypes.hpp>
#include <sturdins/least-squares.hpp>
#include <sturdins/nav-clock.hpp>
#include <thread>

#include "navtools/constants.hpp"
#include "sturdr/structs-enums.hpp"
#include "sturdr/vector-tracking.hpp"

namespace sturdr {

// *=== Navigator ===*
Navigator::Navigator(
    Config &conf, std::shared_ptr<ConcurrentQueue> queue, std::shared_ptr<bool> running)
    : conf_{conf},
      file_size_{conf.general.ms_chunk_size * (uint64_t)conf.rfsignal.samp_freq / 1000},
      nav_file_ptr_{0},
      is_init_{false},
      is_vector_{false},
      n_ch_{0},
      queue_{queue},
      running_{running},
      week_{65535},
      receive_time_{std::nan("1")},
      log_{spdlog::get("sturdr-console")},
      nav_log_{std::make_shared<std::ofstream>(
          conf_.general.out_folder + "/" + conf_.general.scenario + "/Navigation_Log.bin",
          std::ios::binary | std::ios::trunc)},
      eph_log_{std::make_shared<std::ofstream>(
          conf_.general.out_folder + "/" + conf_.general.scenario + "/Ephemeris_Log.bin",
          std::ios::binary | std::ios::trunc)} {
  // apply process noise models to Kalman Filter
  sturdins::NavigationClock clk = sturdins::GetNavClock(conf_.navigation.clock_model);
  kf_.SetClockSpec(clk.h0, clk.h1, clk.h2);
  kf_.SetProcessNoise(conf_.navigation.process_std_vel, conf_.navigation.process_std_att);

  // create dds objects
  eprosima::fastdds::dds::DataWriterQos writer_qos = eprosima::fastdds::dds::DATAWRITER_QOS_DEFAULT;
  writer_qos.history().kind = eprosima::fastdds::dds::KEEP_ALL_HISTORY_QOS;
  eprosima::fastdds::dds::TopicDataType *type1 = new NavMessagePubSubType();
  dds_nav_pub_ = dds_node_.CreatePublisher<NavMessage>("sturdr-navigator", type1, writer_qos);
  eprosima::fastdds::dds::TopicDataType *type2 = new ChannelMessagePubSubType();
  dds_channel_pub_ =
      dds_node_.CreatePublisher<ChannelMessage>("sturdr-channels", type2, writer_qos);
  dds_index_ = 0;

  // start thread
  thread_ = std::thread(&Navigator::NavThread, this);
  log_->info("SturDR Navigator initialized");
}

// *=== ~Navigator ===*
Navigator::~Navigator() {
  // log_->trace("~Navigator");
  nav_log_->close();
  eph_log_->close();
  if (thread_.joinable()) {
    thread_.join();
  }
}

// *=== ~NavThread ===*
void Navigator::NavThread() {
  // std::thread dds_thread(&Navigator::LogDDSMsg, this);

  // run the navigator while SturDR is on
  while (queue_->pop(msg_) && *running_) {
    const std::type_info &msg_type = msg_.type();
    // log_->info("navigator popped message (type = {}) ...", msg_type.name());
    // log_->info("is ChannelNavPacket = {} ...", msg_type == typeid(ChannelNavPacket));

    // check for exit signal
    if (msg_type == typeid(void)) {
      if (queue_->IsFinished()) {
        log_->info("void message received, quitting ...", msg_type.name());
        break;
      } else {
        log_->info("void message received, continuing ...", msg_type.name());
        continue;
      }
    }

    // parse specific message type
    if (msg_type == typeid(ChannelNavPacket)) {
      // log_->info("Received ChannelNavPacket ...");
      ChannelNavPacket tmp = std::any_cast<ChannelNavPacket>(msg_);
      ChannelUpdate(tmp);

    } else if (msg_type == typeid(SturdrNavRequest)) {
      // log_->info("Received SturdrNavRequest ...");
      SturdrNavRequest tmp = std::any_cast<SturdrNavRequest>(msg_);
      ms_elapsed_ = tmp.MsElapsed;
      NavUpdate();

    } else if (msg_type == typeid(ChannelEphemPacket)) {
      // log_->info("Received ChannelEphemPacket ...");
      ChannelEphemPacket tmp = std::any_cast<ChannelEphemPacket>(msg_);
      EphemUpdate(tmp);

    } else {
      log_->warn("Invalid type received in queue!");
    }
  }

  // {
  //   std::unique_lock<std::mutex> lock(dds_mtx_);
  //   dds_cv_.notify_all();
  // }
  // dds_thread.join();
  log_->debug("Navigator stopping ...");
}

// *=== ~NavUpdate ===*
void Navigator::NavUpdate() {
  if (!is_vector_) {
    ScalarUpdate();
  }

  // log to dds subscribers
  // std::unique_lock<std::mutex> lock(dds_mtx_);
  // dds_cv_.notify_all();
  LogDDSMsg();
  // if (is_init_) {
  //   // log to console
  //   log_->info(
  //       "\tLLA:\t{:.8f}, {:.8f}, {:.2f}",
  //       navtools::RAD2DEG<> * kf_.phi_,
  //       navtools::RAD2DEG<> * kf_.lam_,
  //       kf_.h_);
  //   // log_->info("\tRPY:\t{:.2f}, {:.2f}, {:.2f}", rpy(0), rpy(1), rpy(2));
  //   // log_->debug("\tNEDV:\t{:.3f}, {:.3f}, {:.3f}", kf_.vn_, kf_.ve_, kf_.vd_);
  //   // log_->debug(
  //   //     "\tQ:\t{:.4f}, {:.4f}, {:.4f}, {:.4f}",
  //   //     kf_.q_b_l_(0),
  //   //     kf_.q_b_l_(1),
  //   //     kf_.q_b_l_(2),
  //   //     kf_.q_b_l_(3));
  //   // log_->debug("\tCLK:\t{:.2f}, {:.3f}", kf_.cb_, kf_.cd_);
  // }
}

// *=== ~ChannelUpdate ===*
void Navigator::ChannelUpdate(ChannelNavPacket &msg) {
  std::unique_lock<std::mutex> lock(*msg.mtx);
  if (ch_data_.find(msg.Header.ChannelNum) != ch_data_.end()) {
    // update map data
    ch_data_[msg.Header.ChannelNum].Header = msg.Header;
    ch_data_[msg.Header.ChannelNum].FilePtr = msg.FilePtr;
    ch_data_[msg.Header.ChannelNum].Week = msg.Week;
    ch_data_[msg.Header.ChannelNum].ToW = msg.ToW;
    ch_data_[msg.Header.ChannelNum].CNo = msg.CNo;
    ch_data_[msg.Header.ChannelNum].Doppler = msg.Doppler;
    ch_data_[msg.Header.ChannelNum].CodePhase = msg.CodePhase;
    ch_data_[msg.Header.ChannelNum].CarrierPhase = msg.CarrierPhase;
    ch_data_[msg.Header.ChannelNum].DllDisc = msg.DllDisc;
    ch_data_[msg.Header.ChannelNum].PllDisc = msg.PllDisc;
    ch_data_[msg.Header.ChannelNum].FllDisc = msg.FllDisc;
    ch_data_[msg.Header.ChannelNum].PsrVar = msg.PsrVar;
    ch_data_[msg.Header.ChannelNum].PsrdotVar = msg.PsrdotVar;
    ch_data_[msg.Header.ChannelNum].PhaseVar = msg.PhaseVar;
    ch_data_[msg.Header.ChannelNum].Beta = msg.Beta;
    ch_data_[msg.Header.ChannelNum].Lambda = msg.Lambda;
    ch_data_[msg.Header.ChannelNum].ChipRate = msg.ChipRate;
    ch_data_[msg.Header.ChannelNum].CarrierFreq = msg.CarrierFreq;
    ch_data_[msg.Header.ChannelNum].PromptCorrelators = msg.PromptCorrelators;
    ch_data_[msg.Header.ChannelNum].HasData = true;
    ch_data_[msg.Header.ChannelNum].ReadyForVT = false;
    ch_data_[msg.Header.ChannelNum].VTCodeRate = msg.VTCodeRate;
    ch_data_[msg.Header.ChannelNum].VTCarrierFreq = msg.VTCarrierFreq;
    ch_data_[msg.Header.ChannelNum].UnitVec = msg.UnitVec;
    ch_data_[msg.Header.ChannelNum].mtx = msg.mtx;
    ch_data_[msg.Header.ChannelNum].cv = msg.cv;
    ch_data_[msg.Header.ChannelNum].update_complete = msg.update_complete;
    ch_data_[msg.Header.ChannelNum].is_vector = msg.is_vector;
  } else {
    // add new channel to map
    ch_data_.insert(
        {msg.Header.ChannelNum,
         ChannelNavData{
             msg.Header,
             msg.FilePtr,
             satutils::KeplerEphem<double>(),
             msg.Week,
             msg.ToW,
             msg.CNo,
             msg.Doppler,
             0.0,
             msg.CodePhase,
             msg.CarrierPhase,
             msg.DllDisc,
             msg.PllDisc,
             msg.FllDisc,
             msg.PsrVar,
             msg.PsrdotVar,
             msg.PhaseVar,
             msg.Beta,
             msg.Lambda,
             msg.ChipRate,
             msg.CarrierFreq,
             0.0,
             0.0,
             msg.PromptCorrelators,
             false,
             true,
             false,
             msg.VTCodeRate,
             msg.VTCarrierFreq,
             msg.UnitVec,
             msg.mtx,
             msg.cv,
             msg.update_complete,
             msg.is_vector}});
    n_ch_++;
  }

  // notify completion
  if (is_vector_) {
    ch_data_[msg.Header.ChannelNum].ReadyForVT = true;
    // log_->warn(
    //     "ReadyForVT = [{}, {}, {}, {}, {}, {}, {}, {}, {}, {}]",
    //     ch_data_[1].ReadyForVT,
    //     ch_data_[2].ReadyForVT,
    //     ch_data_[3].ReadyForVT,
    //     ch_data_[4].ReadyForVT,
    //     ch_data_[5].ReadyForVT,
    //     ch_data_[6].ReadyForVT,
    //     ch_data_[7].ReadyForVT,
    //     ch_data_[8].ReadyForVT,
    //     ch_data_[9].ReadyForVT,
    //     ch_data_[10].ReadyForVT);
    // log_->warn(
    //     "update_complete = {}, {}",
    //     *msg.update_complete,
    //     *ch_data_[msg.Header.ChannelNum].update_complete);
    if (!VectorUpdate()) return;
  } else {
    *msg.update_complete = true;
    msg.cv->notify_all();
  }
}

// *=== ~EphemUpdate ===*
void Navigator::EphemUpdate(ChannelEphemPacket &msg) {
  if (ch_data_.find(msg.Header.ChannelNum) != ch_data_.end()) {
    // update map data
    ch_data_[msg.Header.ChannelNum].Sv.SetEphemerides(msg.Eph);
    ch_data_[msg.Header.ChannelNum].HasEphem = true;
  } else {
    // add new item to map
    ChannelNavData tmp;
    tmp.HasEphem = true;
    tmp.Sv = satutils::KeplerEphem<double>(msg.Eph);
    ch_data_.insert({msg.Header.ChannelNum, tmp});
    n_ch_++;
  }

  // log ephemeris
  eph_log_->write(reinterpret_cast<char *>(&msg.Header.SVID), sizeof(uint8_t));
  eph_log_->write(reinterpret_cast<char *>(&msg.Eph), sizeof(satutils::KeplerElements<double>));
}

// *=== ScalarUpdate ===*
void Navigator::ScalarUpdate() {
  // make sure enough channels have good data
  std::vector<uint8_t> good_sv;
  uint8_t num_sv = 0;
  for (const std::pair<const uint8_t, sturdr::ChannelNavData> &it : ch_data_) {
    if (it.second.HasData && it.second.HasEphem) {
      good_sv.push_back(it.first);
      num_sv++;
    }
  }
  // log_->info("calling ScalarUpdate (num_sv = {}) ...", (int)num_sv);
  if (num_sv < 4) {
    return;
  }

  // extract navigation information
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
    file_ptrs(i) = ch_data_[it].FilePtr;

    transmit_time(i) = ch_data_[it].ToW + ch_data_[it].CodePhase / ch_data_[it].ChipRate +
                       ch_data_[it].Sv.tgd;  //! FOR GPS L1CA
    ch_data_[it].Sv.CalcNavStates<false>(
        sv_clk, sv_pos.col(i), sv_vel.col(i), sv_acc, transmit_time(i));

    // correct transmit times/drifts for satellite biases (FOR GPS L1CA)
    transmit_time(i) -= sv_clk(0);
    psrdot(i) = -ch_data_[it].Lambda * ch_data_[it].Doppler + navtools::LIGHT_SPEED<> * sv_clk(1);

    // reported variances from tracking loops
    psr_var(i) = ch_data_[it].PsrVar;
    psrdot_var(i) = ch_data_[it].PsrdotVar;
  }

  // Scalar Navigation Update
  if (is_init_) {
    // Kalman Filter
    uint64_t d_samp = GetDeltaSamples(file_ptrs.minCoeff());

    // propagate pseudoranges and rates
    double dt = (static_cast<double>(d_samp) / conf_.rfsignal.samp_freq);
    receive_time_ += dt;
    psr = (receive_time_ - transmit_time.array()) * navtools::LIGHT_SPEED<>;

    // std::cout << "receive time = " << std::setprecision(17) << receive_time_ << " | dt = " << dt
    //           << "\n\n";

    // propagate navigation filter
    kf_.Propagate(dt);

    // measurement corrections
    if (conf_.antenna.is_multi_antenna) {
      // phased array update
      double lamb = ch_data_[1].Lambda;
      Eigen::MatrixXd phase_disc(conf_.antenna.n_ant, num_sv);
      Eigen::MatrixXd phase_disc_var(conf_.antenna.n_ant, num_sv);
      Eigen::VectorXd O = Eigen::VectorXd::Ones(conf_.antenna.n_ant);

      // calculate measured spatial phase
      Eigen::RowVectorXd phase_disc_0 = phase_disc.row(0);
      phase_disc = phase_disc.rowwise() - phase_disc_0;
      for (int i = 0; i < num_sv; i++) {
        for (uint8_t j = 0; j < conf_.antenna.n_ant; j++) {
          navtools::WrapPiToPi<double>(phase_disc(j, i));
        }
      }

      // update
      kf_.PhasedArrayUpdate(
          sv_pos,
          sv_vel,
          psr,
          psrdot,
          phase_disc,
          psr_var,
          psrdot_var,
          phase_disc_var,
          conf_.antenna.ant_xyz,
          conf_.antenna.n_ant,
          lamb);

      // save beamsteering unit vector to channels
      Eigen::Vector3d lla = Eigen::Vector3d{kf_.phi_, kf_.lam_, kf_.h_};
      Eigen::Vector3d nedv = Eigen::Vector3d{kf_.vn_, kf_.ve_, kf_.vd_};
      Eigen::Vector3d xyz = navtools::lla2ecef<double>(lla);
      Eigen::Matrix3d C_e_l = navtools::ecef2nedDcm<double>(lla);
      Eigen::Vector3d xyzv = C_e_l.transpose() * nedv;
      Eigen::Vector3d u, u_ned, tmp;
      double tmp1, tmp2;
      for (size_t ii = 0; ii < ch_data_.size(); ii++) {
        sturdins::RangeAndRate(xyz, xyzv, kf_.cb_, kf_.cd_, sv_pos, sv_vel, u, tmp, tmp1, tmp2);
        u_ned = C_e_l * u;
        // *ch_data_[ii].UnitVec = kf_.C_b_l_.transpose() * u_ned;
        ch_data_[ii].Azimuth = std::atan2(-u_ned(1), -u_ned(0));
        ch_data_[ii].Elevation = -std::asin(u_ned(2));
        ch_data_[ii].Pseudorange = psr[ii];
      }
    } else {
      // standard gnss update
      kf_.GnssUpdate(sv_pos, sv_vel, psr, psrdot, psr_var, psrdot_var);

      Eigen::Vector3d lla = Eigen::Vector3d{kf_.phi_, kf_.lam_, kf_.h_};
      Eigen::Vector3d nedv = Eigen::Vector3d{kf_.vn_, kf_.ve_, kf_.vd_};
      Eigen::Vector3d xyz = navtools::lla2ecef<double>(lla);
      Eigen::Matrix3d C_e_l = navtools::ecef2nedDcm<double>(lla);
      Eigen::Vector3d xyzv = C_e_l.transpose() * nedv;
      Eigen::Vector3d u, u_ned, tmp;
      double tmp1, tmp2;
      for (size_t ii = 0; ii < ch_data_.size(); ii++) {
        sturdins::RangeAndRate(xyz, xyzv, kf_.cb_, kf_.cd_, sv_pos, sv_vel, u, tmp, tmp1, tmp2);
        u_ned = C_e_l * u;
        ch_data_[ii].Azimuth = std::atan2(-u_ned(1), -u_ned(0));
        ch_data_[ii].Elevation = -std::asin(-u_ned(2));
        ch_data_[ii].Pseudorange = psr[ii];
      }
    }

    // update file ptr location
    UpdateFilePtr(d_samp);

  } else {
    // Least Squares
    // initial receive time guess
    receive_time_ = transmit_time.minCoeff() + conf_.navigation.nominal_transit_time;
    psr = (receive_time_ - transmit_time.array()) * navtools::LIGHT_SPEED<>;
    Eigen::VectorXd x{Eigen::Vector<double, 8>::Zero()};
    Eigen::MatrixXd P{Eigen::Matrix<double, 8, 8>::Zero()};
    sturdins::GnssPVT(x, P, sv_pos, sv_vel, psr, psrdot, psr_var, psrdot_var);
    // std::cout << "receive time = " << std::setprecision(17) << receive_time_ << "\n\n";

    // extract solution
    Eigen::Vector3d lla = navtools::ecef2lla<double>(x.segment(0, 3));
    Eigen::Vector3d nedv = navtools::ecef2nedv<double>(x.segment(3, 3), lla);
    receive_time_ -= (x(6) / navtools::LIGHT_SPEED<>);
    x(6) = 0.0;
    // std::cout << "receive time = " << std::setprecision(17) << receive_time_ << "\n\n";
    kf_.SetPosition(lla(0), lla(1), lla(2));
    kf_.SetVelocity(nedv(0), nedv(1), nedv(2));
    kf_.SetClock(x(6), x(7));

    if (conf_.antenna.is_multi_antenna) {
      // Attitude Estimation
      const int N = ch_data_.size();
      double tmp1 = 0.0, tmp2 = 0.0;
      Eigen::Vector3d tmp3, u;
      Eigen::MatrixXd u_ned(3, num_sv);
      Eigen::Matrix3d C_e_l = navtools::ecef2nedDcm<double>(lla);
      Eigen::Matrix3d C_l_b;

      // MUSIC estimator
      Eigen::VectorXd est_az(N), est_el(N);
      for (int i = 0; i < N; i++) {
        // get doa predicted unit vector
        sturdins::MUSIC(
            est_az(i),
            est_el(i),
            ch_data_[i + 1].PromptCorrelators,
            conf_.antenna.ant_xyz,
            static_cast<int>(conf_.antenna.n_ant),
            ch_data_[i + 1].Lambda,
            1e-4);
        // get ephemeris unit vector
        sturdins::RangeAndRate(
            x.segment(0, 3),
            x.segment(3, 3),
            x(6),
            x(7),
            sv_pos.col(i),
            sv_vel.col(i),
            u,
            tmp3,
            tmp1,
            tmp2);
        u_ned.col(i) = C_e_l * u;
      }

      // Wahba's problem solution
      Eigen::MatrixXd u_body_est(3, N);
      Eigen::VectorXd u_body_var{Eigen::VectorXd::Ones(N)};
      u_body_est.row(0) = est_az.array().cos() * est_el.array().cos();
      u_body_est.row(1) = est_az.array().sin() * est_el.array().cos();
      u_body_est.row(2) = -est_el.array().sin();
      // std::cout << "u_body_est = \n" << u_body_est << "\n";
      sturdins::Wahba(C_l_b, u_body_est, u_ned, u_body_var);

      // initialize kalman filter attitude
      // C_l_b << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
      kf_.SetAttitude(C_l_b.transpose());
      // kf_.SetAttitude(C_l_b);

      // save beamsteering unit vector to channels
      for (uint8_t ii = 1; ii <= (uint8_t)N; ii++) {
        *ch_data_[ii].UnitVec = C_l_b * u_ned.col(ii - 1);
        // log_->warn(
        //     "Channel {} - unit_vec = [{}, {}, {}]",
        //     ch_data_[ii].Header.ChannelNum,
        //     (*ch_data_[ii].UnitVec)(0),
        //     (*ch_data_[ii].UnitVec)(1),
        //     (*ch_data_[ii].UnitVec)(2));
        ch_data_[ii].Azimuth = std::atan2(-u_ned(1, ii - 1), -u_ned(0, ii - 1));
        ch_data_[ii].Elevation = -std::asin(-u_ned(2, ii - 1));
        ch_data_[ii].Pseudorange = psr[ii - 1];
      }
    } else {
      Eigen::Matrix3d C_e_l = navtools::ecef2nedDcm<double>(lla);
      Eigen::Vector3d u, u_ned, tmp;
      double tmp1, tmp2;
      for (uint8_t ii = 1; ii <= (uint8_t)ch_data_.size(); ii++) {
        sturdins::RangeAndRate(
            x.segment(0, 3),
            x.segment(3, 3),
            x(6),
            x(7),
            sv_pos.col(ii),
            sv_vel.col(ii),
            u,
            tmp,
            tmp1,
            tmp2);
        u_ned = C_e_l * -u;
        ch_data_[ii].Azimuth = std::atan2(u_ned(1), u_ned(0));
        ch_data_[ii].Elevation = -std::asin(u_ned(2));
        ch_data_[ii].Pseudorange = psr[ii];
      }
    }

    week_ = ch_data_[good_sv[0]].Week + 2048;
    nav_file_ptr_ = file_ptrs.minCoeff();
    is_init_ = true;
    if (conf_.navigation.do_vt) {
      is_vector_ = true;
      // std::cout << "File pointers = [ ";
      for (auto &it : ch_data_) {
        *it.second.is_vector = true;
        *it.second.update_complete = true;
        it.second.cv->notify_all();
        // std::cout << (int)it.second.FilePtr << " ";
      }
      // std::cout << "]\n";
      log_->debug("Channels will now begin vector tracking!");
    }
  }

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

  // log result
  LogNavData();
}

// *=== VectorUpdate ===*
bool Navigator::VectorUpdate() {
  // make sure all channels have arrived
  std::vector<std::pair<uint64_t, uint8_t>> sample_ptrs;
  for (uint8_t i = 1; i <= (uint8_t)ch_data_.size(); i++) {
    // log_->error("{}", ch_data_[i].ReadyForVT);
    if (!ch_data_[i].ReadyForVT) return false;
    // sample_ptrs.push_back({(it.second.FilePtr + nav_file_ptr_) % file_size_, it.first});
    sample_ptrs.push_back({(ch_data_[i].FilePtr + nav_file_ptr_), i});
  }
  // log_->warn(
  //     "calling VectorUpdate, file_ptrs = [{}, {}, {}, {}, {}, {}, {}, {}, {}, {}]...",
  //     sample_ptrs[0].first,
  //     sample_ptrs[1].first,
  //     sample_ptrs[2].first,
  //     sample_ptrs[3].first,
  //     sample_ptrs[4].first,
  //     sample_ptrs[5].first,
  //     sample_ptrs[6].first,
  //     sample_ptrs[7].first,
  //     sample_ptrs[8].first,
  //     sample_ptrs[9].first);

  // known constants
  double intmd_freq_rad = navtools::TWO_PI<> * conf_.rfsignal.intmd_freq;
  double T = 0.02;

  // order updates based on the number of samples they wish to progress
  std::sort(sample_ptrs.begin(), sample_ptrs.end());
  std::vector<int> tmp;
  for (auto &it : ch_data_) {
    tmp.push_back(static_cast<int>(it.second.FilePtr));
  }

  // run updates
  uint64_t d_samp;
  // std::cout << "d_samp = [ ";
  for (std::pair<uint64_t, uint8_t> &p : sample_ptrs) {
    // make sure samples are 'delta' from the last update
    d_samp = GetDeltaSamples(ch_data_[p.second].FilePtr);
    // std::cout << d_samp << ", ";

    // run all vector updates and log to file
    RunVDFllUpdate(
        d_samp,
        conf_.rfsignal.samp_freq,
        intmd_freq_rad,
        ch_data_[p.second],
        receive_time_,
        T,
        kf_,
        conf_.antenna.ant_xyz,
        conf_.antenna.n_ant);

    // update file pointer
    UpdateFilePtr(d_samp);

    // // log solution
    // LogNavData();
  }
  // std::cout << "\n";

  // // MUSIC estimator
  // const int N = ch_data_.size();
  // double est_az = 0, est_el = 0;
  // Eigen::Matrix3Xd u_nav(3, N);
  // Eigen::Matrix3Xd u_est(3, N);
  // Eigen::VectorXd R(N);
  // for (int ii = 0; ii < N; ii++) {
  //   // get doa predicted unit vector
  //   est_az = 0;
  //   est_el = 0;
  //   sturdins::MUSIC(
  //       est_az,
  //       est_el,
  //       ch_data_[ii + 1].PromptCorrelators,
  //       conf_.antenna.ant_xyz,
  //       static_cast<int>(conf_.antenna.n_ant),
  //       ch_data_[ii + 1].Lambda,
  //       1e-4);
  //   u_est(0, ii) = std::cos(est_az) * std::cos(est_el);
  //   u_est(1, ii) = std::sin(est_az) * std::cos(est_el);
  //   u_est(2, ii) = -std::sin(est_el);
  //   R(ii) = ch_data_[ii + 1].PhaseVar;

  //   // get ephemeris unit vector
  //   u_nav.col(ii) = (*ch_data_[ii + 1].UnitVec);
  // }
  // // std::cout << "u_est = \n" << u_est << "\n";
  // // std::cout << "u_nav = \n" << u_nav << "\n";

  // // Wahba's problem solution
  // Eigen::Matrix3d C_l_b;
  // sturdins::Wahba(C_l_b, u_est, u_nav, R);
  // // std::cout << "C_l_b = \n" << C_l_b << "\n";

  // // MUSIC + Wahba's ATTITUDE UPDATE
  // Eigen::Matrix3d R2{Eigen::Matrix3d::Zero()};
  // R2(0, 0) = R.mean() * 0.1;
  // R2(1, 1) = R2(0, 0);
  // R2(2, 2) = R2(0, 0);
  // // std::cout << "R2 = \n" << R2 << "\n";
  // kf_.AttitudeUpdate(C_l_b.transpose(), R2);
  // // kf_.AttitudeUpdate(C_l_b, R2);
  // for (int ii = 1; ii <= N; ii++) {
  //   *ch_data_[ii].UnitVec = kf_.C_b_l_ * (*ch_data_[ii].UnitVec);
  // }

  // log solution only after finished
  LogNavData();

  // tell channels to continue
  // log_->warn("notifying complete ...");
  for (auto &it : ch_data_) {
    it.second.ReadyForVT = false;
    *it.second.update_complete = true;
    it.second.cv->notify_all();
  }

  return true;
}

// *=== LogNavData ===*
void Navigator::LogNavData() {
  Eigen::Vector<double, 11> tmp = kf_.P_.diagonal();
  nav_log_->write(reinterpret_cast<char *>(&ms_elapsed_), sizeof(uint64_t));
  nav_log_->write(reinterpret_cast<char *>(&week_), sizeof(uint16_t));
  nav_log_->write(reinterpret_cast<char *>(&receive_time_), sizeof(double));
  nav_log_->write(reinterpret_cast<char *>(&kf_.phi_), sizeof(double));
  nav_log_->write(reinterpret_cast<char *>(&kf_.lam_), sizeof(double));
  nav_log_->write(reinterpret_cast<char *>(&kf_.h_), sizeof(double));
  nav_log_->write(reinterpret_cast<char *>(&kf_.vn_), sizeof(double));
  nav_log_->write(reinterpret_cast<char *>(&kf_.ve_), sizeof(double));
  nav_log_->write(reinterpret_cast<char *>(&kf_.vd_), sizeof(double));
  nav_log_->write(reinterpret_cast<char *>(kf_.q_b_l_.data()), 4 * sizeof(double));
  nav_log_->write(reinterpret_cast<char *>(&kf_.cb_), sizeof(double));
  nav_log_->write(reinterpret_cast<char *>(&kf_.cd_), sizeof(double));
  nav_log_->write(reinterpret_cast<char *>(tmp.data()), 11 * sizeof(double));
}

void Navigator::LogDDSMsg() {
  // while (*running_) {
  // std::unique_lock<std::mutex> lock(dds_mtx_);
  // dds_cv_.wait(lock);

  dds_index_++;
  auto duration = std::chrono::system_clock::now().time_since_epoch();
  auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
  auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration - seconds);

  // log channel messages
  dds_channel_msg_.stamp().index() = dds_index_;
  dds_channel_msg_.stamp().index(dds_index_);
  dds_channel_msg_.stamp().valid(true);
  dds_channel_msg_.stamp().sec(seconds.count());
  dds_channel_msg_.stamp().nanosec(nanoseconds.count());
  dds_channel_msg_.Week(week_);
  dds_channel_msg_.ToW(receive_time_);
  for (auto &it : ch_data_) {  // (uint8_t)ch_data_.size()
    if (it.first > (uint8_t)ch_data_.size()) {
      continue;
    }
    dds_channel_msg_.ChannelID(it.first - 1);
    dds_channel_msg_.SatelliteID(it.second.Header.SVID);
    dds_channel_msg_.ConstellationID(it.second.Header.Constellation);
    dds_channel_msg_.SignalID(it.second.Header.Signal);
    // dds_channel_msg_.ChannelStatus(it.second.Status);
    dds_channel_msg_.Doppler(-it.second.Lambda * it.second.Doppler);
    dds_channel_msg_.DopplerVariance(it.second.PsrdotVar);
    dds_channel_msg_.Pseudorange(it.second.Pseudorange);
    dds_channel_msg_.PseudorangeVariance(it.second.PsrVar);
    dds_channel_msg_.CarrierPhase(it.second.CarrierPhase);
    dds_channel_msg_.CarrierPhaseVariance(it.second.PhaseVar);
    dds_channel_msg_.CNo(it.second.CNo);
    dds_channel_msg_.Azimuth(it.second.Azimuth * navtools::RAD2DEG<>);
    dds_channel_msg_.Elevation(it.second.Elevation * navtools::RAD2DEG<>);
    // dds_channel_msg_.IE(it.second.IE);
    // dds_channel_msg_.IP(it.second.IP);
    // dds_channel_msg_.IL(it.second.IL);
    // dds_channel_msg_.QE(it.second.QE);
    // dds_channel_msg_.QP(it.second.QP);
    // dds_channel_msg_.QL(it.second.QL);
    // dds_channel_msg_.TapSpace(it.second.tap_space);
    dds_channel_pub_->Publish(dds_channel_msg_);
    // std::cout << "Azimuth = " << dds_channel_msg_.Azimuth()
    //           << ", Elevation = " << dds_channel_msg_.Elevation() << "\n";
  }

  // log navigation message
  if (is_init_) {
    Eigen::Vector3d rpy = navtools::quat2euler<double>(kf_.q_b_l_, true) * navtools::RAD2DEG<>;
    dds_nav_msg_.stamp().index(dds_index_);
    dds_nav_msg_.stamp().valid(true);
    dds_nav_msg_.stamp().sec(seconds.count());
    dds_nav_msg_.stamp().nanosec(nanoseconds.count());
    dds_nav_msg_.Week(week_);
    dds_nav_msg_.ToW(receive_time_);
    dds_nav_msg_.Lat(navtools::RAD2DEG<> * kf_.phi_);
    dds_nav_msg_.Lon(navtools::RAD2DEG<> * kf_.lam_);
    dds_nav_msg_.H(kf_.h_);
    dds_nav_msg_.Vn(kf_.vn_);
    dds_nav_msg_.Ve(kf_.ve_);
    dds_nav_msg_.Vd(kf_.vd_);
    dds_nav_msg_.Roll(rpy(0));
    dds_nav_msg_.Pitch(rpy(1));
    dds_nav_msg_.Yaw(rpy(2));
    dds_nav_msg_.Bias(kf_.cb_);
    dds_nav_msg_.Drift(kf_.cd_);
    dds_nav_msg_.P0(kf_.P_(0, 0));
    dds_nav_msg_.P1(kf_.P_(1, 1));
    dds_nav_msg_.P2(kf_.P_(2, 2));
    dds_nav_msg_.P3(kf_.P_(3, 3));
    dds_nav_msg_.P4(kf_.P_(4, 4));
    dds_nav_msg_.P5(kf_.P_(5, 5));
    dds_nav_msg_.P6(kf_.P_(6, 6));
    dds_nav_msg_.P7(kf_.P_(7, 7));
    dds_nav_msg_.P8(kf_.P_(8, 8));
    dds_nav_msg_.P9(kf_.P_(9, 9));
    dds_nav_msg_.P10(kf_.P_(10, 10));
    dds_nav_pub_->Publish(dds_nav_msg_);

    // log to console
    log_->info(
        "\tLLA:\t{:.8f}, {:.8f}, {:.2f}",
        navtools::RAD2DEG<> * kf_.phi_,
        navtools::RAD2DEG<> * kf_.lam_,
        kf_.h_);
    log_->info("\tRPY:\t{:.2f}, {:.2f}, {:.2f}", rpy(0), rpy(1), rpy(2));
    // log_->debug("\tNEDV:\t{:.3f}, {:.3f}, {:.3f}", kf_.vn_, kf_.ve_, kf_.vd_);
    // log_->debug(
    //     "\tQ:\t{:.4f}, {:.4f}, {:.4f}, {:.4f}",
    //     kf_.q_b_l_(0),
    //     kf_.q_b_l_(1),
    //     kf_.q_b_l_(2),
    //     kf_.q_b_l_(3));
    // log_->debug("\tCLK:\t{:.2f}, {:.3f}", kf_.cb_, kf_.cd_);
  }
  // }
}

// *=== UpdateFilePtr ===*
void Navigator::UpdateFilePtr(const uint64_t &d_samp) {
  nav_file_ptr_ += d_samp;
  nav_file_ptr_ %= file_size_;
}

// *=== GetDeltaSamples ===*
uint64_t Navigator::GetDeltaSamples(const uint64_t &new_file_ptr) {
  if (nav_file_ptr_ <= new_file_ptr) {
    return new_file_ptr - nav_file_ptr_;
  } else {
    return file_size_ - nav_file_ptr_ + new_file_ptr;
  }
}

}  // namespace sturdr