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
    spdlog::get("sturdr-console")->error("ephemeris.cpp CheckTime failed! Error -> {}", e.what());
    return std::nan("1");
  }
}

}  // namespace sturdr