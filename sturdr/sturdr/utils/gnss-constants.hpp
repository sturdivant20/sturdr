/**
 * *gnss-constants.hpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/utils/gnss-constants.hpp
 * @brief   GNSS constants.
 * @date    December 2024
 * @author  Daniel Sturdivant <sturdivant20@gmail.com>
 * @ref     1. "Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems", 2nd
 *              Edition, 2013 - Groves
 *          2. "IS-GPS-200N", 2022
 * =======  ========================================================================================
 */

#ifndef STURDR_GNSS_CONSTANTS_HPP
#define STURDR_GNSS_CONSTANTS_HPP

#include <cstdint>

namespace sturdr {

// Powers of 2
inline static const double TWO_P4 = std::pow(2.0, 4);
inline static const double TWO_N5 = std::pow(2.0, -5);
inline static const double TWO_N19 = std::pow(2.0, -19);
inline static const double TWO_N29 = std::pow(2.0, -29);
inline static const double TWO_N31 = std::pow(2.0, -31);
inline static const double TWO_N33 = std::pow(2.0, -33);
inline static const double TWO_N43 = std::pow(2.0, -43);
inline static const double TWO_N55 = std::pow(2.0, -55);

//! ----- GPS --------------------------------------------------------------------------------------

inline static const double GPS_PI = 3.1415926535898;  // GPS defined pi constant
inline static const double GPS_WEEK = 604800.0;       // seconds in a week
inline static const double GPS_HALF_WEEK = 302400.0;  // seconds in half a week

inline static const double GPS_L1CA_CARRIER_FREQ = 1575.42e6;
inline static const double GPS_L1CA_CODE_FREQ = 1.023e6;
inline static const uint16_t GPS_L1CA_CODE_MS = 1;
inline static const uint16_t GPS_L1CA_CODE_SIZE = 1023;
inline static const uint16_t LNAV_PREAMBLE_SIZE = 8;
inline static const uint16_t LNAV_MS_PER_BIT = 20;
inline static const uint16_t LNAV_SUBFRAME_SIZE = 300;
inline static const uint16_t LNAV_WORD_SIZE = 30;
inline static const uint8_t LNAV_PREAMBLE_BITS = 0b10001011;
inline static const uint8_t LNAV_INV_PREAMBLE_BITS = 0b01110100;

//! ----- GALILEO ----------------------------------------------------------------------------------
//! ----- GLONASS ----------------------------------------------------------------------------------
//! ----- BEIDOU -----------------------------------------------------------------------------------
//! ----- QZSS -------------------------------------------------------------------------------------
//! ----- NAVIC ------------------------------------------------------------------------------------

}  // end namespace sturdr

#endif