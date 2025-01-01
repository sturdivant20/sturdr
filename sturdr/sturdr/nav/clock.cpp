/**
 * *clock.cpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/nav/clock.cpp
 * @brief   Implementation and definitions for calculating clock parameters.
 * @date    December 2024
 * @author  Daniel Sturdivant <sturdivant20@gmail.com>
 * =======  ========================================================================================
 */

#include "sturdr/nav/clock.hpp"

#include <navtools/constants.hpp>

namespace sturdr {

// *=== GetNavClock ===*
NavigationClock GetNavClock(std::string clock_name) {
  for (char &c : clock_name) {
    c = std::tolower(c);
  };
  if (clock_name == "low_quality_tcxo") {
    return LOW_QUALITY_TCXO;
  } else if (clock_name == "high_quality_tcxo") {
    return HIGH_QUALITY_TCXO;
  } else if (clock_name == "ocxo") {
    return OCXO;
  } else if (clock_name == "rubidium") {
    return RUBIDIUM;
  } else if (clock_name == "cesium") {
    return CESIUM;
  } else {
    // Default to low quality TCXO
    return LOW_QUALITY_TCXO;
  }
}

// *=== NavClockDynamics ===*
Eigen::Matrix2d NavClockDynamics(const double &T) {
  return (Eigen::Matrix2d() << 1.0, T, 0.0, 1.0).finished();
}

// *=== NavClockProcessCov ===*
Eigen::Matrix2d NavClockProcessCov(const NavigationClock &c, const double &T) {
  double LS2 = navtools::LIGHT_SPEED<double> * navtools::LIGHT_SPEED<double>;
  double T2 = T * T;
  double T3 = T * T2;
  double q_bb = LS2 * ((c.h0 * T / 2.0) + (2.0 * c.h1 * T2) +
                       (2.0 / 3.0 * navtools::PI_SQU<double> * c.h2 * T3));
  double q_bd = LS2 * ((2.0 * c.h1 * T) + (navtools::PI_SQU<double> * c.h2 * T2));
  double q_dd =
      LS2 * ((c.h0 / 2.0 / T) + (2.0 * c.h1) + (8.0 / 3.0 * navtools::PI_SQU<double> * c.h2 * T));
  return (Eigen::Matrix2d() << q_bb, q_bd, q_bd, q_dd).finished();
}

}  // end namespace sturdr
