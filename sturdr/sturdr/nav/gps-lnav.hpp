/**
 * *gps-lnav.hpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/nav/gps-lnav.hpp
 * @brief   Implementation of GPS L1 C/A navigation message parsing.
 * @date    December 2024
 * @author  Daniel Sturdivant <sturdivant20@gmail.com>
 * @ref     1. "IS-GPS-200N", 2022
 *          2. "A Software-Defined GPS and Galileo Receiver: A Single-Frequency Approach", 2007
 *              - Borre, Akos, Bertelsen, Rinder, Jensen
 * =======  ========================================================================================
 */

#pragma once

#ifndef STURDR_GPS_LNAV_HPP
#define STURDR_GPS_LNAV_HPP

#include <vector>

#include "sturdr/nav/ephemeris.hpp"

namespace sturdr {

class GpsLnavParser {
 public:
  /**
   * *=== GpsLnavParser ===*
   * @brief Constructor
   */
  GpsLnavParser();

  /**
   * *=== ~GpsLnavParser ===*
   * @brief Destructor
   */
  ~GpsLnavParser();

  /**
   * *=== NextBit ===*
   * @brief Read the next navigation data bit
   * @param bit Navigation data bit parsed by tracking loop
   * @return True|False based on if a subframe was successfully parsed
   */
  bool NextBit(const bool &bit);

  /**
   * *=== ParseSubframe ===*
   * @brief Attempts to parse subframe
   * @return True|False based on if a subframe was successfully parsed
   */
  bool ParseSubframe();

  /**
   * *=== ParityCheck ===*
   * @brief IS-GPS-200N pg. 139
   * @param gpsword GPS word to evaluate
   * @param D29star
   * @param D30star
   * @return Parity success or failure
   */
  bool ParityCheck(const uint32_t &gpsword, const bool &D29star, const bool &D30star);

  /**
   * *=== LoadPreamble ===*
   * @brief Reads words 1 and 2 of each subframe (IS-GPS-200N pg. 92)
   */
  void LoadPreamble();

  /**
   * *=== LoadSubframe1 ===*
   * @brief Reads GPS LNAV subframe 1 (IS-GPS-200N pg. 80 & 97)
   */
  void LoadSubframe1();

  /**
   * *=== LoadSubframe2 ===*
   * @brief Reads GPS LNAV subframe 2 (IS-GPS-200N pg. 81 & 105)
   */
  void LoadSubframe2();

  /**
   * *=== LoadSubframe3 ===*
   * @brief Reads GPS LNAV subframe 3 (IS-GPS-200N pg. 82 & 105)
   */
  void LoadSubframe3();

  /**
   * *=== LoadSubframe4 ===*
   * @brief Reads GPS LNAV subframe 4 (IS-GPS-200N pg. )
   */
  void LoadSubframe4();

  /**
   * *=== LoadSubframe5 ===*
   * @brief Reads GPS LNAV subframe 5 (IS-GPS-200N pg. )
   */
  void LoadSubframe5();

  /**
   * *=== GetEphemerides ===*
   * @returns Current set of ephemerides
   */
  Ephemerides GetEphemerides();

  /**
   * *=== GetWeekNumber ===*
   * @returns Week number
   */
  uint16_t GetWeekNumber();

  /**
   * *=== GetTimeOfWeek ===*
   * @returns GPS time of week [gps seconds]
   */
  double GetTimeOfWeek();

  /**
   * *=== AreEphemeridesParsed ===*
   * @returns True|False based on if subframe 1,2 and 3 have been parsed
   */
  bool AreEphemeridesParsed();

 private:
  bool preamble_sync_{false};
  bool sub1_parsed_{false};
  bool sub2_parsed_{false};
  bool sub3_parsed_{false};
  uint32_t subframe[10];
  uint32_t prev_32_bits_{0};
  uint16_t word_cnt_{0};
  uint16_t bit_cnt_{0};
  uint16_t bits_since_preamble_{301};
  std::vector<uint16_t> preamble_idx_;
  uint16_t week_;
  double ToW_;
  Ephemerides eph_;
};

}  // end namespace sturdr

#endif