/**
 * *ephemeris.hpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/ephemeris.hpp
 * @brief   Satellite ephemeris navigation module.
 * @date    December 2024
 * @author  Daniel Sturdivant <sturdivant20@gmail.com>
 * @ref     1. "IS-GPS-200N", 2022
 *          2. "A Software-Defined GPS and Galileo Receiver: A Single-Frequency Approach", 2007
 *              - Borre, Akos, Bertelsen, Rinder, Jensen
 * =======  ========================================================================================
 */

#pragma once

#ifndef STURDR_EPHEMERIS_HPP
#define STURDR_EPHEMERIS_HPP

#include <fmt/format.h>
#include <spdlog/spdlog.h>

#include <Eigen/Dense>
#include <cmath>
#include <exception>
#include <navtools/constants.hpp>

#include "sturdr/gnss-constants.hpp"

namespace sturdr {

// TODO: not all of these are double!!
/**
 * @brief Ephemerides found in navigation message
 */
struct Ephemerides {
  double iode{std::nan("1")};      // Issue of data Ephemeris
  double iodc{std::nan("1")};      // Issue of data Clock
  double toe{std::nan("1")};       // Time of Ephemeris
  double toc{std::nan("1")};       // Time of Clock
  double tgd{std::nan("1")};       // Group delay
  double af2{std::nan("1")};       // 2nd order clock correction coef.
  double af1{std::nan("1")};       // 1st order clock correction coef.
  double af0{std::nan("1")};       // 0th order clock correction coef.
  double e{std::nan("1")};         // Eccentricity
  double sqrtA{std::nan("1")};     // Square root of semi-major axis
  double deltan{std::nan("1")};    // Mean motion difference
  double m0{std::nan("1")};        // Mean anomaly
  double omega0{std::nan("1")};    // Longitude of ascending node
  double omega{std::nan("1")};     // Argument of perigee
  double omegaDot{std::nan("1")};  // Rate of right ascension
  double i0{std::nan("1")};        // Inclination angle
  double iDot{std::nan("1")};      // Rate of inclination angle
  double cuc{std::nan("1")};       // Cos-harmonic correction coef. to the argument of latitude
  double cus{std::nan("1")};       // Sin-harmonic correction coef. to the argument of latitude
  double cic{std::nan("1")};       // Cos-harmonic correction coef. to the angle of inclination
  double cis{std::nan("1")};       // Sin-harmonic correction coef. to the angle of inclination
  double crc{std::nan("1")};       // Cos-harmonic correction coef. to the orbit radius
  double crs{std::nan("1")};       // Sin-harmonic correction coef. to the orbit radius
  double ura{std::nan("1")};       // Estimated accuracy
  double health{std::nan("1")};    // Satellite health
};

/**
 * *=== CheckTime ===*
 * @brief Ensure that the time is within a GPS week
 * @param t Time to check [gps seconds]
 */
double CheckTime(double t);

/**
 * *=== GetSvNavStates ===
 * @brief Calculates satellite position, velocity, and acceleration given ephemeris elements.
 * @param clk           Satellite clock corrections vector
 * @param pos           Satellite position vector
 * @param vel           Satellite velocity vector
 * @param acc           Satellite acceleration vector
 * @param transmit_time GPS system/transmitter time (TOW) of the satellite (accounting for transit
 *                      time from satellite to receiver) [gps seconds]
 * @param eph           Ephemerides
 * @param calc_acc      Boolean of whether to calculate satellite accelerations or not
 * @return True|False based on success
 */
template <bool calc_acc = false>
bool GetSvNavStates(
    Eigen::Ref<Eigen::Vector3d> clk,
    Eigen::Ref<Eigen::Vector3d> pos,
    Eigen::Ref<Eigen::Vector3d> vel,
    Eigen::Ref<Eigen::Vector3d> acc,
    const double& transmit_time,
    const Ephemerides& eph) {
  // bool GetSvNavStates(
  //     Eigen::Vector3d &clk,
  //     Eigen::Vector3d &pos,
  //     Eigen::Vector3d &vel,
  //     Eigen::Vector3d &acc,
  //     const double &transmit_time,
  //     const Ephemerides &eph) {
  try {
    // Constants
    double A = eph.sqrtA * eph.sqrtA;
    double n0 = std::sqrt(navtools::WGS84_MU<double> / (A * A * A));  // computed mean motion
    double n = n0 + eph.deltan;                                       // corrected mean motion
    double E2 = eph.e * eph.e;                                        // eccentricity squared
    double SQ1ME2 = std::sqrt(1.0 - E2);                              // common eccentricity factor

    // satellite clock correction (sv time)
    double dt = CheckTime(transmit_time - eph.toc);          // time from clock epoch
    double dt_sv = eph.af0 + dt * (eph.af1 + dt * eph.af2);  // group delay depends on frequency
    double tk = CheckTime(transmit_time - dt_sv - eph.toe);  // corrected time difference

    // mean anomaly
    double Mk = std::fmod(eph.m0 + n * tk + navtools::TWO_PI<double>, navtools::TWO_PI<double>);

    // calculate eccentric anomaly
    double COSE, SINE, dE;
    double Ek = Mk;
    for (int i = 0; i < 10; i++) {
      COSE = std::cos(Ek);  // cosine of eccentric anomaly
      SINE = std::sin(Ek);  // sine of eccentric anomaly
      dE = (Mk - Ek + eph.e * SINE) / (1.0 - eph.e * COSE);
      if (std::abs(dE) < 1e-15) {
        break;
      }
      Ek += dE;
    }
    Ek = std::fmod(Ek + navtools::TWO_PI<double>, navtools::TWO_PI<double>);
    double DEN = 1.0 - eph.e * COSE;  // common denominator

    // true anomaly
    // double vk = 2.0 * np.atan2(np.sqrt((1.0 + e) / (1.0 - e)) * np.tan(0.5 * Ek), 1.0);
    double vk = std::atan2(SQ1ME2 * SINE, COSE - eph.e);

    // argument of latitude
    double Phik = std::fmod(vk + eph.omega, navtools::TWO_PI<double>);
    double COS2PHI = std::cos(2.0 * Phik);
    double SIN2PHI = std::sin(2.0 * Phik);

    // corrections
    double uk = Phik + (eph.cus * SIN2PHI + eph.cuc * COS2PHI);     // argument of latitude
    double rk = A * DEN + (eph.crs * SIN2PHI + eph.crc * COS2PHI);  // radius
    double ik = eph.i0 + eph.iDot * tk + (eph.cis * SIN2PHI + eph.cic * COS2PHI);  // inclination
    double wk = std::fmod(
        eph.omega0 +
            tk * (eph.omegaDot -
                  navtools::WGS84_OMEGA<double>)-(navtools::WGS84_OMEGA<double> * eph.toe) +
            navtools::TWO_PI<double>,
        navtools::TWO_PI<double>);  // longitude of ascending node - (omega == w)
    double COSU = std::cos(uk);
    double SINU = std::sin(uk);
    double COSI = std::cos(ik);
    double SINI = std::sin(ik);
    double COSW = std::cos(wk);
    double SINW = std::sin(wk);

    // derivatives
    double EDotk = n / DEN;               // eccentric anomaly rate
    double vDotk = EDotk * SQ1ME2 / DEN;  // true anomaly rate
    double iDotk =
        eph.iDot + 2.0 * vDotk * (eph.cis * COS2PHI - eph.cic * SIN2PHI);  // inclination angle rate
    double uDotk =
        vDotk * (1.0 + 2.0 * (eph.cus * COS2PHI - eph.cuc * SIN2PHI));  // argument of latitude rate
    double rDotk = (eph.e * A * EDotk * SINE) +
                   2.0 * vDotk * (eph.crs * COS2PHI - eph.crc * SIN2PHI);  // radius rate
    double wDotk =
        eph.omegaDot - navtools::WGS84_OMEGA<double>;  // longitude of ascending node rate

    // position calculations
    double xk_orb = rk * COSU;  // x-position in orbital frame
    double yk_orb = rk * SINU;  // y-position in orbital frame
    pos(0) = xk_orb * COSW - yk_orb * COSI * SINW;
    pos(1) = xk_orb * SINW + yk_orb * COSI * COSW;
    pos(2) = yk_orb * SINI;

    // velocity calculations
    double xDotk_orb = rDotk * COSU - rk * uDotk * SINU;  // x-velocity in orbital frame
    double yDotk_orb = rDotk * SINU + rk * uDotk * COSU;  // y-velocity in orbital frame
    vel(0) = -(xk_orb * wDotk * SINW) + (xDotk_orb * COSW) - (yDotk_orb * SINW * COSI) -
             (yk_orb * (wDotk * COSW * COSI - iDotk * SINW * SINI));
    vel(1) = (xk_orb * wDotk * COSW) + (xDotk_orb * SINW) + (yDotk_orb * COSW * COSI) -
             (yk_orb * (wDotk * SINW * COSI + iDotk * COSW * SINI));
    vel(2) = (yDotk_orb * SINI) + (yk_orb * iDotk * COSI);

    // relativistic clock calculations (user time)
    double FESQA = navtools::F<double> * eph.e * eph.sqrtA;  // relativistic time factor
    // clk(0) = dt_sv + (FESQA * SINE);
    clk(0) = dt_sv -
             2.0 * pos.dot(vel) / (navtools::LIGHT_SPEED<double> * navtools::LIGHT_SPEED<double>);
    clk(1) = eph.af1 + (2.0 * eph.af2 * dt) + (n * FESQA * COSE / DEN);

    if constexpr (calc_acc) {
      double F = -1.5 * navtools::J2<double> * (navtools::WGS84_MU<double> / (rk * rk)) *
                 std::pow(navtools::WGS84_R0<double> / rk, 2);
      double TMP1 = -navtools::WGS84_MU<double> / (rk * rk * rk);
      double TMP2 = 5.0 * std::pow(pos(2) / rk, 2);
      double TMP3 = navtools::WGS84_OMEGA<double> * navtools::WGS84_OMEGA<double>;

      // state
      acc(0) = TMP1 * pos(0) + F * (1.0 - TMP2) * (pos(0) / rk) +
               2.0 * vel(1) * navtools::WGS84_OMEGA<double> + pos(0) * TMP3;
      acc(1) = TMP1 * pos(1) + F * (1.0 - TMP2) * (pos(1) / rk) -
               2.0 * vel(0) * navtools::WGS84_OMEGA<double> + pos(1) * TMP3;
      acc(2) = TMP1 * pos(2) + F * (3.0 - TMP2) * (pos(2) / rk);

      // clock
      clk(2) = 2.0 * eph.af2 - (n * n * FESQA * SINE / (DEN * DEN));
    }
    return true;
  } catch (std::exception& e) {
    spdlog::get("sturdr-console")
        ->error("ephemeris.cpp GetSvNavStates failed! Error -> {}", e.what());
    return false;
  }
};

}  // namespace sturdr

//! ------------------------------------------------------------------------------------------------

inline std::ostream& operator<<(std::ostream& os, const sturdr::Ephemerides& c) {
  os << "Ephemerides: \n";
  os << "\tiode     = " << c.iode << "\n";
  os << "\tiodc     = " << c.iodc << "\n";
  os << "\ttoe      = " << c.toe << "\n";
  os << "\ttoc      = " << c.toc << "\n";
  os << "\ttgd      = " << c.tgd << "\n";
  os << "\taf2      = " << c.af2 << "\n";
  os << "\taf1      = " << c.af1 << "\n";
  os << "\taf0      = " << c.af0 << "\n";
  os << "\te        = " << c.e << "\n";
  os << "\tsqrtA    = " << c.sqrtA << "\n";
  os << "\tdeltan   = " << c.deltan << "\n";
  os << "\tm0       = " << c.m0 << "\n";
  os << "\tomega0   = " << c.omega0 << "\n";
  os << "\tomega    = " << c.omega << "\n";
  os << "\tomegaDot = " << c.omegaDot << "\n";
  os << "\ti0       = " << c.i0 << "\n";
  os << "\tiDot     = " << c.iDot << "\n";
  os << "\tcuc      = " << c.cuc << "\n";
  os << "\tcus      = " << c.cus << "\n";
  os << "\tcic      = " << c.cic << "\n";
  os << "\tcis      = " << c.cis << "\n";
  os << "\tcrc      = " << c.crc << "\n";
  os << "\tcrs      = " << c.crs << "\n";
  os << "\tura      = " << c.ura << "\n";
  os << "\thealth   = " << c.health << "\n";
  return os;
};
template <>
struct fmt::formatter<sturdr::Ephemerides> : formatter<string_view> {
  auto format(sturdr::Ephemerides& c, format_context& ctx) const;
};
inline auto fmt::formatter<sturdr::Ephemerides>::format(
    sturdr::Ephemerides& c, format_context& ctx) const {
  std::ostringstream oss;
  oss << "Ephemerides: \n";
  oss << "\tiode     = " << c.iode << "\n";
  oss << "\tiodc     = " << c.iodc << "\n";
  oss << "\ttoe      = " << c.toe << "\n";
  oss << "\ttoc      = " << c.toc << "\n";
  oss << "\ttgd      = " << c.tgd << "\n";
  oss << "\taf2      = " << c.af2 << "\n";
  oss << "\taf1      = " << c.af1 << "\n";
  oss << "\taf0      = " << c.af0 << "\n";
  oss << "\te        = " << c.e << "\n";
  oss << "\tsqrtA    = " << c.sqrtA << "\n";
  oss << "\tdeltan   = " << c.deltan << "\n";
  oss << "\tm0       = " << c.m0 << "\n";
  oss << "\tomega0   = " << c.omega0 << "\n";
  oss << "\tomega    = " << c.omega << "\n";
  oss << "\tomegaDot = " << c.omegaDot << "\n";
  oss << "\ti0       = " << c.i0 << "\n";
  oss << "\tiDot     = " << c.iDot << "\n";
  oss << "\tcuc      = " << c.cuc << "\n";
  oss << "\tcus      = " << c.cus << "\n";
  oss << "\tcic      = " << c.cic << "\n";
  oss << "\tcis      = " << c.cis << "\n";
  oss << "\tcrc      = " << c.crc << "\n";
  oss << "\tcrs      = " << c.crs << "\n";
  oss << "\tura      = " << c.ura << "\n";
  oss << "\thealth   = " << c.health;
  return formatter<string_view>::format(oss.str(), ctx);
};

#endif