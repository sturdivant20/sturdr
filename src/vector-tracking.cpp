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

#include <spdlog/fmt/ostr.h>
#include <spdlog/spdlog.h>

#include <Eigen/Dense>
#include <navtools/constants.hpp>
#include <navtools/math.hpp>
#include <sturdins/least-squares.hpp>

#include "navtools/frames.hpp"

template <typename T>
struct fmt::formatter<T, std::enable_if_t<std::is_base_of_v<Eigen::DenseBase<T>, T>, char>>
    : ostream_formatter {};

namespace sturdr {

// *=== VectorDllNco ===*
double VectorDllNco(double &chip_rate, double &T, double &tau, double &tR, double &tR_pred) {
  return (chip_rate * T - tau) / (tR_pred - tR);
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
    sturdins::Kns &filt,
    const Eigen::Ref<const Eigen::MatrixXd> &ant_xyz,
    const int &n_ant) {
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
  psrdot_var << data.PsrdotVar;

  // 4. Combine vector and scaler measurements
  //! SUBTRACTED CODE DISCRIMINATOR BECAUSE MY EARLY AND LATE CORRELATORS ARE REVERSED FROM PAPERS
  psr(0) -= data.Beta * data.DllDisc;
  psrdot(0) -= data.Lambda * data.FllDisc;

  // 4. Run kalman filter (save predicted nav-state)
  filt.Propagate(dt);
  if (n_ant > 1) {
    // if using antenna array - estimate attitude
    Eigen::VectorXd phase_disc = (-data.PllDisc).array() + data.PllDisc(0);
    Eigen::VectorXd phase_disc_var(n_ant);
    for (int j = 0; j < n_ant; j++) {
      navtools::WrapPiToPi<double>(phase_disc(j));
      phase_disc_var(j) = data.PhaseVar;
    }

    // spdlog::get("sturdr-console")->error("phase_disc: {}", phase_disc.transpose());

    filt.PhasedArrayUpdate(
        sv_pos,
        sv_vel,
        psr,
        psrdot,
        phase_disc,
        psr_var,
        psrdot_var,
        phase_disc_var,
        ant_xyz,
        n_ant,
        data.Lambda);
  } else {
    filt.GnssUpdate(sv_pos, sv_vel, psr, psrdot, psr_var, psrdot_var);
  }

  // 5. predict state at end of next code period (correct satellite pos/tR)
  Eigen::Vector3d pos_pred, vel_pred;
  double cb_pred, cd_pred;
  filt.FalsePropagateState(pos_pred, vel_pred, cb_pred, cd_pred, T);
  double tT_pred = data.ToW + T + data.Sv.tgd;  //! FOR GPS L1CA

  // 6. Predict measurements
  Eigen::Vector3d u, udot;
  double psr_pred, psrdot_pred;
  data.Sv.CalcNavStates<false>(sv_clk, sv_pos, sv_vel, sv_acc, tT_pred);
  sturdins::RangeAndRate(
      pos_pred, vel_pred, cb_pred, cd_pred, sv_pos, sv_vel, u, udot, psr_pred, psrdot_pred);
  double tR_pred = tT_pred - sv_clk(0) + (psr_pred / navtools::LIGHT_SPEED<>);
  psrdot_pred -= navtools::LIGHT_SPEED<> * sv_clk(1);

  // 7. Unit Vector for potential beamsteering (in body frame)
  Eigen::Vector3d lla{filt.phi_, filt.lam_, filt.h_};
  Eigen::Matrix3d C_e_l = navtools::ecef2nedDcm<double>(lla);
  *data.VTUnitVec = filt.C_b_l_.transpose() * (C_e_l * u);

  // 8. Vector FLL update
  *data.VTCarrierFreq = VectorFllNco(intmd_freq, data.Lambda, psrdot_pred);
  //! this is for appending a PLL after VFLL
  // *data.VTCarrierFreq = (psrdot(0) - psrdot_pred) / data.Lambda;

  // 9. Vector DLL update
  *data.VTCodeRate = VectorDllNco(data.ChipRate, T, data.CodePhase, tR, tR_pred);
}

}  // namespace sturdr