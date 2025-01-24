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

#ifndef STURDR_NAVIGATOR_HPP
#define STURDR_NAVIGATOR_HPP

#include <spdlog/spdlog.h>

#include <Eigen/Dense>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <sturdins/kns.hpp>
#include <thread>

#include "sturdr/concurrent-queue.hpp"
#include "sturdr/structs-enums.hpp"

namespace sturdr {

class Navigator {
 private:
  /**
   * @brief navigation parameters
   */
  Config conf_;
  uint16_t week_;
  double receive_time_;
  Eigen::Vector3d lla_;
  Eigen::Vector3d nedv_;
  double cb_;
  double cd_;
  double T_;
  bool is_initialized_;
  sturdins::Kns kf_;

  /**
   * @brief thread scheduling
   */
  std::map<uint8_t, ChannelNavData> channel_data_;
  std::shared_ptr<ConcurrentQueue<ChannelNavPacket>> nav_queue_;
  std::shared_ptr<ConcurrentQueue<ChannelEphemPacket>> eph_queue_;
  std::condition_variable cv_;
  std::mutex mtx_, event_mtx_;
  bool update_;
  std::shared_ptr<bool> running_;
  std::thread thread_;

  /**
   * @brief spdlog loggers
   */
  std::shared_ptr<spdlog::logger> log_;
  //   std::shared_ptr<spdlog::logger> nav_log_;
  //   std::shared_ptr<spdlog::logger> eph_log_;

 public:
  /**
   * *=== Navigator ===*
   * @brief constructor
   * @param conf  SturDR config
   */
  Navigator(
      Config &conf,
      std::shared_ptr<ConcurrentQueue<ChannelNavPacket>> &nav_queue,
      std::shared_ptr<ConcurrentQueue<ChannelEphemPacket>> &eph_queue,
      std::shared_ptr<bool> &running);

  /**
   * *=== ~Navigator ===*
   * @brief destructor
   */
  ~Navigator();

  /**
   * *=== Notify ===*
   * @brief Notifys the waiting navigation thread to block the queues and perform an update
   */
  void Notify();

 private:
  /**
   * *=== NavigationThread ===*
   * @brief Runs the thread controlling navigation inputs and outputs
   */
  void NavigationThread();

  /**
   * *=== ChannelNavPacketListener ===*
   * @brief Listens to the channels for navigation packet outputs
   */
  void ChannelNavPacketListener();

  /**
   * *=== ChannelEphemPacketListener ===*
   * @brief Listens to the channels for ephemeris packet outputs
   */
  void ChannelEphemPacketListener();

  /**
   * *=== InitNavSolution ===*
   * @brief initializes navigation with least squares
   */
  void InitNavSolution(
      const Eigen::Ref<const Eigen::MatrixXd> &sv_pos,
      const Eigen::Ref<const Eigen::MatrixXd> &sv_vel,
      const Eigen::Ref<const Eigen::VectorXd> &transmit_time,
      const Eigen::Ref<const Eigen::VectorXd> &psrdot,
      const Eigen::Ref<const Eigen::VectorXd> &psr_var,
      const Eigen::Ref<const Eigen::VectorXd> &psrdot_var);

  /**
   * *=== ScalarNavSolution ===*
   * @brief propagates navigation with 'scalar' navigation techniques
   */
  void ScalarNavSolution(
      const Eigen::Ref<const Eigen::MatrixXd> &sv_pos,
      const Eigen::Ref<const Eigen::MatrixXd> &sv_vel,
      const Eigen::Ref<const Eigen::VectorXd> &transmit_time,
      const Eigen::Ref<const Eigen::VectorXd> &psrdot,
      const Eigen::Ref<const Eigen::VectorXd> &psr_var,
      const Eigen::Ref<const Eigen::VectorXd> &psrdot_var);

  /**
   * *=== VectorNavSolution ===*
   * @brief propagates navigation with 'vector' navigation techniques
   */
  void VectorNavSolution();
};

}  // namespace sturdr

#endif