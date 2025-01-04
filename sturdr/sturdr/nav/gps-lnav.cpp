/**
 * *gps-lnav.cpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/nav/gps-lnav.cpp
 * @brief   Implementation of GPS L1 C/A navigation message parsing.
 * @date    December 2024
 * @author  Daniel Sturdivant <sturdivant20@gmail.com>
 * @ref     1. "IS-GPS-200N", 2022
 *          2. "A Software-Defined GPS and Galileo Receiver: A Single-Frequency Approach", 2007
 *              - Borre, Akos, Bertelsen, Rinder, Jensen
 * =======  ========================================================================================
 */

#include "sturdr/nav/gps-lnav.hpp"

#include <exception>
#include <navtools/binary-ops.hpp>

#include "sturdr/utils/gnss-constants.hpp"

namespace sturdr {
// clang-format off
inline static const uint8_t GPS_D25[14] = {2,3,4,6,7,11,12,13,14,15,18,19,21,24};   // [1,2,3,5,6,10,11,12,13,14,17,18,20,23]
inline static const uint8_t GPS_D26[14] = {3,4,5,7,8,12,13,14,15,16,19,20,22,25};   // [2,3,4,6,7,11,12,13,14,15,18,19,21,24]
inline static const uint8_t GPS_D27[14] = {2,4,5,6,8,9,13,14,15,16,17,20,21,23};    // [1,3,4,5,7,8,12,13,14,15,16,19,20,22]
inline static const uint8_t GPS_D28[14] = {3,5,6,7,9,10,14,15,16,17,18,21,22,24};   // [2,4,5,6,8,9,13,14,15,16,17,20,21,23]
inline static const uint8_t GPS_D29[15] = {2,4,6,7,8,10,11,15,16,17,18,19,22,23,25};// [1,3,5,6,7,9,10,14,15,16,17,18,21,22,24]
inline static const uint8_t GPS_D30[13] = {4,6,7,9,10,11,12,14,16,20,23,24,25};     // [3,5,6,8,9,10,11,13,15,19,22,23,24]
// clang-format on

// *=== GpsLnavParser ===*
GpsLnavParser::GpsLnavParser() {
}

// *=== ~GpsLnavParser ===*
GpsLnavParser::~GpsLnavParser() {
}

// *=== NextBit ===*
bool GpsLnavParser::NextBit(const bool &bit) {
  // data bits ordered [-2 -1 0 ... 29]
  bool subframe_parsed = false;

  // shift prev 32 to the left and insert next bit
  prev_32_bits_ <<= 1;
  navtools::ModifyBit<false>(prev_32_bits_, 31, bit);
  bit_cnt_ += 1;

  // Step 1: find preamble
  if (!preamble_sync_) {
    // Check most recent 8 bits for a preamble
    uint8_t test = static_cast<uint8_t>(prev_32_bits_ & 0x000000FF);
    if ((test == LNAV_PREAMBLE_BITS) || (test == LNAV_INV_PREAMBLE_BITS)) {
      if (bits_since_preamble_ == 301) {
        // If here, an initial preamble has been detected
        bit_cnt_ = 8;
        word_cnt_ = 0;
        bits_since_preamble_ = 0;
      } else if (bits_since_preamble_ == 300) {
        // If here, a second preamble has been successfully detected 300 bits apart!
        preamble_sync_ = true;
        subframe_parsed = ParseSubframe();
        bit_cnt_ = 8;
        word_cnt_ = 0;
        bits_since_preamble_ = 0;
      } else {
        // There is another preamble detection here, log it
        preamble_idx_.push_back(bits_since_preamble_);
      }
    }

    // Check if preamble failed to sync
    if (bits_since_preamble_ == 300) {
      // Sync failed, reset to next detection
      uint16_t bits_to_shift = (preamble_idx_[0] % 30) + 1;
      uint16_t words_to_shift = preamble_idx_[0] / 30;

      // shift down full words
      if (words_to_shift > 0) {
        for (uint16_t i = 0; i < (10 - words_to_shift); i++) {
          subframe[i] = subframe[i + words_to_shift];
        }
      }

      // shift down remaining bits
      if (bits_to_shift > 0) {
        uint32_t curr_bits, next_bits;
        for (uint16_t i = 0; i < (9 - words_to_shift); i++) {
          curr_bits = ((subframe[i] & 0xFFFFFFFC) << (bits_to_shift - 1));
          next_bits = (subframe[i + 1] >> (31 - bits_to_shift));
          subframe[i] = curr_bits | next_bits;
        }
      }

      // update counters
      bits_since_preamble_ -= preamble_idx_[0];
      bit_cnt_ = 31 - bits_to_shift + 8;  // +8 from preamble
      word_cnt_ = 9 - words_to_shift;

      // git rid of used detection and update detection counter
      for (uint16_t &x : preamble_idx_) {
        x -= preamble_idx_[0];
      }
      preamble_idx_.erase(preamble_idx_.begin());
    }

    // increment bit counter
    bits_since_preamble_++;
  }

  // Step 2: Save words
  if (bit_cnt_ == 30) {
    subframe[word_cnt_] = prev_32_bits_;
    bit_cnt_ = 0;
    word_cnt_ += 1;
  }

  // Step 3: Parse subframe
  if (word_cnt_ == 10) {
    word_cnt_ = 0;
    if (preamble_sync_) {
      subframe_parsed = ParseSubframe();
    }
  }

  return subframe_parsed;
}

// *=== ParseSubframe ===*
bool GpsLnavParser::ParseSubframe() {
  // data bits ordered [-2 -1 0 ... 29]

  // Step 1: Validate received data bits
  bool D29star, D30star;
  for (uint16_t i = 0; i < 10; i++) {
    D29star = navtools::CheckBit<false>(subframe[i], 0);
    D30star = navtools::CheckBit<false>(subframe[i], 0);

    // check bit polarity
    if (D30star) subframe[i] ^= 0x3FFFFFC0;

    // check parity
    if (!ParityCheck(subframe[i], D29star, D30star)) {
      throw std::runtime_error("GpsLnavParser::ParseSubframe Invalid parity check!");
    }
  }

  // Step 2: Get subframe id
  uint8_t sub_id = static_cast<uint8_t>((subframe[1] & 0x00000700) >> 8);

  // Step 3: Parse subframe
  bool subframe_parsed = false;
  switch (sub_id) {
    case 1:
      LoadSubframe1();
      sub1_parsed_ = true;
      subframe_parsed = true;
      break;
    case 2:
      LoadSubframe2();
      sub2_parsed_ = true;
      subframe_parsed = true;
      break;
    case 3:
      LoadSubframe3();
      sub3_parsed_ = true;
      subframe_parsed = true;
      break;
    case 4:
      subframe_parsed = true;
      break;
    case 5:
      subframe_parsed = true;
      break;
    default:
      throw std::range_error("GpsLnavParser::ParseSubframe Invalid subframe ID!");
      break;
  }

  return subframe_parsed;
}

// *=== ParityCheck ===*
bool GpsLnavParser::ParityCheck(const uint32_t &gpsword, const bool &D29star, const bool &D30star) {
  // Calculate the parity
  uint32_t parity = 0;
  navtools::ModifyBit<false>(parity, 26, D29star ^ navtools::MultiXor<false>(gpsword, GPS_D25));
  navtools::ModifyBit<false>(parity, 27, D30star ^ navtools::MultiXor<false>(gpsword, GPS_D26));
  navtools::ModifyBit<false>(parity, 28, D29star ^ navtools::MultiXor<false>(gpsword, GPS_D27));
  navtools::ModifyBit<false>(parity, 29, D30star ^ navtools::MultiXor<false>(gpsword, GPS_D28));
  navtools::ModifyBit<false>(parity, 30, D30star ^ navtools::MultiXor<false>(gpsword, GPS_D29));
  navtools::ModifyBit<false>(parity, 31, D29star ^ navtools::MultiXor<false>(gpsword, GPS_D30));

  // compare the parity
  if (parity == (gpsword & 0x0000003F)) {
    return true;
  }
  return false;
}

// *=== LoadPreamble ===*
void GpsLnavParser::LoadPreamble() {
  // word 1
  // tlm_message = static_cast<uint16_t>((subframe[0] & 0x003FFF00) >> 8);        // bits 9-22
  // integrity_status_flag = static_cast<bool>((subframe[0] & 0x00000080) >> 7);  // bit 23

  // word 2
  ToW_ = 6.0 * static_cast<double>((subframe[1] & 0x3FFFE000) >> 13);  // bits 1-17
  // alert_flag = static_cast<bool>((subframe[1] & 0x00001000) >> 12);       // bit 18
  // anti_spoof_flag = static_cast<bool>((subframe[1] & 0x00000800) >> 11);  // bit 19
}

// *=== LoadSubframe1 ===*
void GpsLnavParser::LoadSubframe1() {
  uint32_t tmp1, tmp2;

  // Word 1-2
  LoadPreamble();

  // Word 3
  week_ = static_cast<uint16_t>((subframe[2] & 0x3FF00000) >> 20);  // bits 1-10
  // l2_flag = static_cast<uint8_t>((subframe[2] & 0x000C0000) >> 18);     // bits 11-12
  eph_.ura = static_cast<uint8_t>((subframe[2] & 0x0003C000) >> 14);    // bits 13-16
  eph_.health = static_cast<uint8_t>((subframe[2] & 0x00003F00) >> 8);  // bits 17-22

  // Word 7
  tmp1 = (subframe[6] & 0x00003FC0) >> 6;
  eph_.tgd = navtools::TwosComp(tmp1, 8) * TWO_N31;  // bits 17-24

  // Word 8
  tmp1 = (subframe[2] & 0x000000C0) >> 6;                                    // bits 23-24 (word 3)
  tmp2 = (subframe[7] & 0x3FC00000) >> 22;                                   // bits 1-8
  eph_.iodc = static_cast<double>((tmp1 << 8) | tmp2);                       //
  eph_.toc = static_cast<double>((subframe[7] & 0x003FFFC0) >> 6) * TWO_P4;  // bits 9-24

  // Word 9
  tmp1 = (subframe[8] & 0x000000C0) >> 22;
  tmp2 = (subframe[8] & 0x003FFFC0) >> 6;
  eph_.af2 = navtools::TwosComp(tmp1, 8) * TWO_N55;   // bits 1-8
  eph_.af1 = navtools::TwosComp(tmp2, 16) * TWO_N43;  // bits 9-24

  // word 10
  tmp1 = (subframe[9] & 0x3FFFFF00) >> 8;
  eph_.af0 = navtools::TwosComp(tmp1, 22) * TWO_N31;  // bits 1-22
}

// *=== LoadSubframe2 ===*
void GpsLnavParser::LoadSubframe2() {
  uint32_t tmp1, tmp2, tmp3;

  // Word 1-2
  LoadPreamble();

  // Word 3
  tmp3 = (subframe[2] & 0x003FFFC0) >> 6;
  eph_.iode = static_cast<double>((subframe[2] & 0x3FC00000) >> 22);  // bits 1-8
  eph_.crs = navtools::TwosComp(tmp3, 16) * TWO_N5;                   // bits 9-24

  // Word 4-5
  tmp3 = (subframe[3] & 0x3FFFC000) >> 14;
  eph_.deltan = navtools::TwosComp(tmp3, 16) * GPS_PI * TWO_N43;  // bits 1-16
  tmp1 = (subframe[3] & 0x00003FC0) >> 6;                         // bits 17-24
  tmp2 = (subframe[4] & 0x3FFFFFC0) >> 6;                         // bits 1-24
  tmp3 = (tmp1 << 24) | tmp2;
  eph_.m0 = navtools::TwosComp(tmp3, 32) * GPS_PI * TWO_N31;

  // Word 6 and 7
  tmp3 = (subframe[5] & 0x3FFFC000) >> 14;
  eph_.cuc = navtools::TwosComp(tmp3, 16) * TWO_N29;  // bits 1-16
  tmp1 = (subframe[5] & 0x00003FC0) >> 6;             // bits 17-24
  tmp2 = (subframe[6] & 0x3FFFFFC0) >> 6;             // bits 1-24
  eph_.e = static_cast<double>((tmp1 << 24) | tmp2) * TWO_N33;

  // Word 8 and 9
  tmp3 = (subframe[7] & 0x3FFFC000) >> 14;
  eph_.cus = navtools::TwosComp(tmp3, 16) * TWO_N29;  // bits 1-16
  tmp1 = (subframe[7] & 0x00003FC0) >> 6;             // bits 17-24
  tmp2 = (subframe[8] & 0x3FFFFFC0) >> 6;             // bits 1-24
  eph_.sqrtA = static_cast<double>((tmp1 << 24) | tmp2) * TWO_N19;

  // Word 10
  eph_.toe = static_cast<double>((subframe[9] & 0x3FFFC000) >> 14) * TWO_P4;  // bits 1-16
  // fit_interval_alert_flag = bool((subframe[9] & 0x00002000) >> 13);           // bit 17
}

// *=== LoadSubframe3 ===*
void GpsLnavParser::LoadSubframe3() {
  uint32_t tmp1, tmp2, tmp3;

  // Word 1-2
  LoadPreamble();

  // word 3 and 4
  tmp3 = (subframe[2] & 0x3FFFC000) >> 14;
  eph_.cic = navtools::TwosComp(tmp3, 16) * TWO_N29;  // bits 1-16
  tmp1 = (subframe[2] & 0x00003FC0) >> 6;             // bits 17-24
  tmp2 = (subframe[3] & 0x3FFFFFC0) >> 6;             // bits 1-24
  tmp3 = (tmp1 << 24) | tmp2;
  eph_.omega0 = navtools::TwosComp(tmp3, 32) * GPS_PI * TWO_N31;

  // Word 5 and 6
  tmp3 = (subframe[4] & 0x3FFFC000) >> 14;
  eph_.cis = navtools::TwosComp(tmp3, 16) * TWO_N29;  // bits 1-16
  tmp1 = (subframe[4] & 0x00003FC0) >> 6;             // bits 17-24
  tmp2 = (subframe[5] & 0x3FFFFFC0) >> 6;             // bits 1-24
  tmp3 = (tmp1 << 24) | tmp2;
  eph_.i0 = navtools::TwosComp(tmp3, 32) * GPS_PI * TWO_N31;

  // word 7 and 8
  tmp3 = (subframe[6] & 0x3FFFC000) >> 14;
  eph_.crc = navtools::TwosComp(tmp3, 16) * TWO_N5;  // bits 1-16
  tmp1 = (subframe[6] & 0x00003FC0) >> 6;            // bits 17-24
  tmp2 = (subframe[7] & 0x3FFFFFC0) >> 6;            // bits 1-24
  tmp3 = (tmp1 << 24) | tmp2;
  eph_.omega = navtools::TwosComp(tmp3, 32) * GPS_PI * TWO_N31;

  // Word 9
  tmp3 = (subframe[8] & 0x3FFFFFC0) >> 6;
  eph_.omegaDot = navtools::TwosComp(tmp3, 24) * GPS_PI * TWO_N43;  // bits 1-24

  // Word 10
  eph_.iode = static_cast<double>((subframe[9] & 0x3FC00000) >> 22);  // bits 1-8
  tmp3 = (subframe[9] & 0x003FFF00) >> 8;
  eph_.iDot = navtools::TwosComp(tmp3, 14) * GPS_PI * TWO_N43;  // bits 9-22
}

// *=== LoadSubframe4 ===*
void GpsLnavParser::LoadSubframe4() {
}

// *=== LoadSubframe5 ===*
void GpsLnavParser::LoadSubframe5() {
}

// *=== GetEphemerides ===*
Ephemerides GpsLnavParser::GetEphemerides() {
  return eph_;
}

// *=== GetWeekNumber ===*
uint16_t GpsLnavParser::GetWeekNumber() {
  return week_;
}

// *=== GetTimeOfWeek ===*
double GpsLnavParser::GetTimeOfWeek() {
  return ToW_;
}

// *=== AreEphemeridesParsed ===*
bool GpsLnavParser::AreEphemeridesParsed() {
  return (sub1_parsed_ & sub2_parsed_ & sub3_parsed_);
}

}  // end namespace sturdr