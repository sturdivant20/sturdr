/**
 * *ephemeris.cpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/nav/ephemeris.cpp
 * @brief   Satellite ephemeris navigation module.
 * @date    December 2024
 * @author  Daniel Sturdivant <sturdivant20@gmail.com>
 * @ref     1. "IS-GPS-200N", 2022
 *          2. "A Software-Defined GPS and Galileo Receiver: A Single-Frequency Approach", 2007
 *              - Borre, Akos, Bertelsen, Rinder, Jensen
 * =======  ========================================================================================
 */

#include "sturdr/nav/ephemeris.hpp"

#include <spdlog/spdlog.h>

#include <exception>
#include <navtools/constants.hpp>

#include "sturdr/utils/gnss-constants.hpp"

namespace sturdr {

// *=== CheckTime ===*
double CheckTime(double t) {
  try {
    if (t > GPS_HALF_WEEK) {
      t -= GPS_WEEK;
    } else if (t < -GPS_HALF_WEEK) {
      t += GPS_WEEK;
    }
    return t;
  } catch (std::exception &e) {
    spdlog::default_logger()->error("ephemeris.cpp CheckTime failed! Error -> {}", e.what());
    return std::nan("1");
  }
}

// // *=== GetSvNavStates ===
// template <bool calc_acc>
// bool GetSvNavStates(
//     Eigen::Ref<Eigen::Vector3d> clk,
//     Eigen::Ref<Eigen::Vector3d> pos,
//     Eigen::Ref<Eigen::Vector3d> vel,
//     Eigen::Ref<Eigen::Vector3d> acc,
//     const double &transmit_time,
//     const Ephemerides &eph) {
//   try {
//     // Constants
//     double A = eph.sqrtA * eph.sqrtA;
//     double n0 = std::sqrt(navtools::WGS84_MU<double> / (A * A * A));  // computed mean motion
//     double n = n0 + eph.deltan;                                       // corrected mean motion
//     double E2 = eph.e * eph.e;                                        // eccentricity squared
//     double SQ1ME2 = std::sqrt(1.0 - E2);                              // common eccentricity
//     factor

//     // satellite clock correction (sv time)
//     double dt = CheckTime(transmit_time - eph.toc);          // time from clock epoch
//     double dt_sv = eph.af0 + dt * (eph.af1 + dt * eph.af2);  // group delay depends on frequency
//     double tk = CheckTime(transmit_time - dt_sv - eph.toe);  // corrected time difference

//     // mean anomaly
//     double Mk = std::fmod(eph.m0 + n * tk + navtools::TWO_PI<double>, navtools::TWO_PI<double>);

//     // calculate eccentric anomaly
//     double COSE, SINE, dE;
//     double Ek = Mk;
//     for (int i = 0; i < 10; i++) {
//       COSE = std::cos(Ek);  // cosine of eccentric anomaly
//       SINE = std::sin(Ek);  // sine of eccentric anomaly
//       dE = (Mk - Ek + eph.e * SINE) / (1.0 - eph.e * COSE);
//       if (std::abs(dE) < 1e-15) {
//         break;
//       }
//       Ek += dE;
//     }
//     Ek = std::fmod(Ek + navtools::TWO_PI<double>, navtools::TWO_PI<double>);
//     double DEN = 1.0 - eph.e * COSE;  // common denominator

//     // true anomaly
//     // double vk = 2.0 * np.atan2(np.sqrt((1.0 + e) / (1.0 - e)) * np.tan(0.5 * Ek), 1.0);
//     double vk = std::atan2(SQ1ME2 * SINE, COSE - eph.e);

//     // argument of latitude
//     double Phik = std::fmod(vk + eph.omega, navtools::TWO_PI<double>);
//     double COS2PHI = std::cos(2.0 * Phik);
//     double SIN2PHI = std::sin(2.0 * Phik);

//     // corrections
//     double uk = Phik + (eph.cus * SIN2PHI + eph.cuc * COS2PHI);     // argument of latitude
//     double rk = A * DEN + (eph.crs * SIN2PHI + eph.crc * COS2PHI);  // radius
//     double ik = eph.i0 + eph.iDot * tk + (eph.cis * SIN2PHI + eph.cic * COS2PHI);  // inclination
//     double wk = std::fmod(
//         eph.omega0 +
//             tk * (eph.omegaDot -
//                   navtools::WGS84_OMEGA<double>)-(navtools::WGS84_OMEGA<double> * eph.toe) +
//             navtools::TWO_PI<double>,
//         navtools::TWO_PI<double>);  // longitude of ascending node - (omega == w)
//     double COSU = std::cos(uk);
//     double SINU = std::sin(uk);
//     double COSI = std::cos(ik);
//     double SINI = std::sin(ik);
//     double COSW = std::cos(wk);
//     double SINW = std::sin(wk);

//     // derivatives
//     double EDotk = n / DEN;               // eccentric anomaly rate
//     double vDotk = EDotk * SQ1ME2 / DEN;  // true anomaly rate
//     double iDotk =
//         eph.iDot + 2.0 * vDotk * (eph.cis * COS2PHI - eph.cic * SIN2PHI);  // inclination angle
//         rate
//     double uDotk =
//         vDotk * (1.0 + 2.0 * (eph.cus * COS2PHI - eph.cuc * SIN2PHI));  // argument of latitude
//         rate
//     double rDotk = (eph.e * A * EDotk * SINE) +
//                    2.0 * vDotk * (eph.crs * COS2PHI - eph.crc * SIN2PHI);  // radius rate
//     double wDotk =
//         eph.omegaDot - navtools::WGS84_OMEGA<double>;  // longitude of ascending node rate

//     // position calculations
//     double xk_orb = rk * COSU;  // x-position in orbital frame
//     double yk_orb = rk * SINU;  // y-position in orbital frame
//     pos(0) = xk_orb * COSW - yk_orb * COSI * SINW;
//     pos(1) = xk_orb * SINW + yk_orb * COSI * COSW;
//     pos(2) = yk_orb * SINI;

//     // velocity calculations
//     double xDotk_orb = rDotk * COSU - rk * uDotk * SINU;  // x-velocity in orbital frame
//     double yDotk_orb = rDotk * SINU + rk * uDotk * COSU;  // y-velocity in orbital frame
//     vel(0) = -(xk_orb * wDotk * SINW) + (xDotk_orb * COSW) - (yDotk_orb * SINW * COSI) -
//              (yk_orb * (wDotk * COSW * COSI - iDotk * SINW * SINI));
//     vel(1) = (xk_orb * wDotk * COSW) + (xDotk_orb * SINW) + (yDotk_orb * COSW * COSI) -
//              (yk_orb * (wDotk * SINW * COSI + iDotk * COSW * SINI));
//     vel(2) = (yDotk_orb * SINI) + (yk_orb * iDotk * COSI);

//     // relativistic clock calculations (user time)
//     double FESQA = navtools::F<double> * eph.e * eph.sqrtA;  // relativistic time factor
//     clk(0) = dt_sv + (FESQA * SINE);
//     // dt_sv - 2.0 * pos.dot(vel) / (navtools::LIGHT_SPEED<double> *
//     navtools::LIGHT_SPEED<double>); clk(1) = eph.af1 + (2.0 * eph.af2 * dt) + (n * FESQA * COSE /
//     DEN);

//     if constexpr (calc_acc) {
//       double F = -1.5 * navtools::J2<double> * (navtools::WGS84_MU<double> / (rk * rk)) *
//                  std::pow(navtools::WGS84_R0<double> / rk, 2);
//       double TMP1 = -navtools::WGS84_MU<double> / (rk * rk * rk);
//       double TMP2 = 5.0 * std::pow(pos(2) / rk, 2);
//       double TMP3 = navtools::WGS84_OMEGA<double> * navtools::WGS84_OMEGA<double>;

//       // state
//       acc(0) = TMP1 * pos(0) + F * (1.0 - TMP2) * (pos(0) / rk) +
//                2.0 * vel(1) * navtools::WGS84_OMEGA<double> + pos(0) * TMP3;
//       acc(1) = TMP1 * pos(1) + F * (1.0 - TMP2) * (pos(1) / rk) -
//                2.0 * vel(0) * navtools::WGS84_OMEGA<double> + pos(1) * TMP3;
//       acc(2) = TMP1 * pos(2) + F * (3.0 - TMP2) * (pos(2) / rk);

//       // clock
//       clk(2) = 2.0 * eph.af2 - (n * n * FESQA * SINE / (DEN * DEN));
//     }
//     return true;
//   } catch (std::exception &e) {
//     spdlog::default_logger()->error("ephemeris.cpp GetSvNavStates failed! Error -> {}",
//     e.what()); return false;
//   }
// }

}  // namespace sturdr