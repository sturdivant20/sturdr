/**
 * *vector-tracking.cpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/vector-tracking.cpp
 * @brief   Handles key vector tracking functionality
 * @date    January 2025
 * @ref     1. "GPS Carrier Phase Tracking in Difficult Environments Using Vector Tracking For
 *              Precise Positioning and Vehicle Attitude Estimation" (Ph.D. Dissertation, Auburn
 *              University) - Scott Martin
 *          2. "Modeling and Performance Analysis of GPS Vector Tracking Algorithms" (Ph.D
 *              Dissertation, Auburn University) - Matthew Lashley
 *          3. "A GPS and GLONASS L1 Vector Tracking Software-Defined Receiver" (Masters Thesis,
 *              Auburn University) - Tanner Watts
 *          4. "A GPS L5 Software Defined Vector Tracking Receiver" (Masters Thesis, Auburn
 *              University) - C. Anderson Givhan
 * =======  ========================================================================================
 */

#include "sturdr/vector-tracking.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <spdlog/spdlog.h>

#include <Eigen/Dense>
#include <iomanip>
#include <navtools/constants.hpp>
#include <sturdins/least-squares.hpp>

namespace sturdr {

// *=== VectorDllNco ===*
double VectorDllNco(double &chip_rate, double &T, double &theta, double &tR, double &tR_pred) {
  return (chip_rate * T - theta) / (tR_pred - tR);
  // return (chip_rate * T) / (tR_pred - tR);
}

// *=== VectorFllNco ===*
double VectorFllNco(double &intmd_freq, double &lambda, double &psrdot) {
  return intmd_freq - (psrdot / lambda);
}

// *=== RunVDFllUpdate ===*
void RunVDFllUpdate(
    uint64_t &d_samp,
    double &samp_freq,
    double &intmd_freq,
    ChannelNavData &data,
    double &tR,
    double &T,
    sturdins::Kns &filt) {
  // 1. Estimate satellite position, velocity, and clock from transmit time
  Eigen::Vector3d sv_pos, sv_vel, sv_clk, sv_acc;
  double tT = data.ToW + data.CodePhase / data.ChipRate + data.Sv.tgd;  //! FOR GPS L1CA
  data.Sv.CalcNavStates<false>(sv_clk, sv_pos, sv_vel, sv_acc, tT);

  // 2. Propage receive time by number of samples accumulated since last vector update
  double dt = (static_cast<double>(d_samp) / samp_freq);
  tR += dt;

  // 3. Estimate scalar-tracking based measurements
  tT -= sv_clk(0);
  Eigen::VectorXd psr(1);
  psr << (tR - tT) * navtools::LIGHT_SPEED<>;
  Eigen::VectorXd psrdot(1);
  psrdot << -data.Lambda * data.Doppler + navtools::LIGHT_SPEED<> * sv_clk(1);
  Eigen::VectorXd psr_var(1);
  psr_var << data.PsrVar;
  Eigen::VectorXd psrdot_var(1);
  psrdot_var << 1000.0 * data.PsrdotVar;

  // 4. Combine vector and scaler measurements
  // spdlog::get("sturdr-console")
  //     ->warn(
  //         "tR: {}, psr: {:.1f}, psrdot: {:.3f}, Dll: {:.3f}, Fll: {:.3f}",
  //         tR,
  //         psr(0),
  //         psrdot(0),
  //         data.Beta * data.DllDisc,
  //         -data.Lambda * data.FllDisc);
  psr(0) -= data.Beta * data.DllDisc;  //! I DONT KNOW WHY THIS NEED TO BE SUBTRACTED
  // psrdot(0) -= data.Lambda * data.FllDisc;

  // 4. Run kalman filter (save predicted nav-state)
  filt.Propagate(dt);
  filt.GnssUpdate(sv_pos, sv_vel, psr, psrdot, psr_var, psrdot_var);

  // 5. predict state at end of next code period (correct satellite pos/tR)
  Eigen::Vector3d pos_pred, vel_pred;
  double cb_pred, cd_pred;
  filt.FalsePropagateState(pos_pred, vel_pred, cb_pred, cd_pred, T);
  double tT_pred = data.ToW + T + data.Sv.tgd;  //! FOR GPS L1CA
  // spdlog::get("sturdr-console")
  //     ->info(
  //         "x_pred - {:.2f}, {:.2f}, {:.2f}, {:.4f}, {:.4f}, {:.4f}, {:.2f}, {:.4f}",
  //         pos_pred(0),
  //         pos_pred(1),
  //         pos_pred(2),
  //         vel_pred(0),
  //         vel_pred(1),
  //         vel_pred(3),
  //         cb_pred,
  //         cd_pred);

  // 6. Predict measurements
  Eigen::Vector3d u, udot;
  double psr_pred, psrdot_pred;
  data.Sv.CalcNavStates<false>(sv_clk, sv_pos, sv_vel, sv_acc, tT_pred);
  sturdins::RangeAndRate(
      pos_pred, vel_pred, cb_pred, cd_pred, sv_pos, sv_vel, u, udot, psr_pred, psrdot_pred);
  double tR_pred = tT_pred - sv_clk(0) + (psr_pred / navtools::LIGHT_SPEED<>);
  psrdot_pred -= navtools::LIGHT_SPEED<> * sv_clk(1);
  spdlog::get("sturdr-console")
      ->warn(
          "psr: {:.1f}, psrdot: {:.3f}, Dll: {:.3f}, Fll: {:.3f}, psr_p: {:.1f}, psrdot_p: {:.3f}",
          psr(0),
          psrdot(0),
          data.Beta * data.DllDisc,
          -data.Lambda * data.FllDisc,
          psr_pred,
          psrdot_pred);
  // spdlog::get("sturdr-console")->warn("pred - psr: {}, psrdot: {}", psr_pred, psrdot_pred);

  // 7. Vector FLL update
  *data.VTCarrierFreq = VectorFllNco(intmd_freq, data.Lambda, psrdot_pred);
  // *data.VTCarrierFreq = (psrdot(0) - psrdot_pred) / data.Lambda;

  // 8. Vector DLL update
  // spdlog::get("sturdr-console")
  //     ->info(
  //         "chip_rate: {}, theta: {}, T: {}, dtR: {}",
  //         data.ChipRate,
  //         data.CodePhase,
  //         T,
  //         tR_pred - tR);
  *data.VTCodeRate = VectorDllNco(data.ChipRate, T, data.CodePhase, tR, tR_pred);
}

}  // namespace sturdr