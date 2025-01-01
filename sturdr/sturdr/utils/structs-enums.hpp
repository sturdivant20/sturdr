/**
 * *structs-enums.hpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/utils/structs-enums.hpp
 * @brief   Common structs and enum definitions used across STURDR.
 * @date    December 2024
 * @author  Daniel Sturdivant <sturdivant20@gmail.com>
 * =======  ========================================================================================
 */

#ifndef STURDR_STRUCTS_ENUMS_HPP
#define STURDR_STRUCTS_ENUMS_HPP

#include <spdlog/spdlog.h>

#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

namespace sturdr {

namespace GnssSystem {
enum GnssSystem {
  UNKNOWN = 0,
  GPS = 1,
  GALILEO = 2,
  GLONASS = 3,
  BEIDOU = 4,
  QZSS = 5,
  IRNSS = 6,
  SBAS = 7
};
std::ostream& operator<<(std::ostream& os, const GnssSystem Token) {
  switch (Token) {
    case GnssSystem::UNKNOWN:
      os << std::string_view("UNKNOWN");
      break;
    case GnssSystem::GPS:
      os << std::string_view("GPS");
      break;
    case GnssSystem::GALILEO:
      os << std::string_view("GALILEO");
      break;
    case GnssSystem::GLONASS:
      os << std::string_view("GLONASS");
      break;
    case GnssSystem::BEIDOU:
      os << std::string_view("BEIDOU");
      break;
    case GnssSystem::QZSS:
      os << std::string_view("QZSS");
      break;
    case GnssSystem::IRNSS:
      os << std::string_view("IRNSS");
      break;
    case GnssSystem::SBAS:
      os << std::string_view("SBAS");
      break;
  }
  return os;
};
};  // namespace GnssSystem

namespace GnssSignal {
enum GnssSignal {
  UNKNOWN = 0,
  GPS_L1CA = 1,
  GPS_L1C = 2,
  GPS_L2C = 3,
  GPS_L5 = 4,
  GALILEO_E1 = 5,
  GALILEO_E6 = 6,
  GALILEO_E5 = 7,
  GALILEO_E5a = 8,
  GALILEO_E5b = 9
};
std::ostream& operator<<(std::ostream& os, const GnssSignal Token) {
  switch (Token) {
    case GnssSignal::UNKNOWN:
      os << std::string_view("UNKNOWN");
      break;
    case GnssSignal::GPS_L1CA:
      os << std::string_view("GPS_L1CA");
      break;
    case GnssSignal::GPS_L1C:
      os << std::string_view("GPS_L1C");
      break;
    case GnssSignal::GPS_L2C:
      os << std::string_view("GPS_L2C");
      break;
    case GnssSignal::GPS_L5:
      os << std::string_view("GPS_L5");
      break;
    case GnssSignal::GALILEO_E1:
      os << std::string_view("GALILEO_E1");
      break;
    case GnssSignal::GALILEO_E6:
      os << std::string_view("GALILEO_E6");
      break;
    case GnssSignal::GALILEO_E5:
      os << std::string_view("GALILEO_E5");
      break;
    case GnssSignal::GALILEO_E5a:
      os << std::string_view("GALILEO_E5a");
      break;
    case GnssSignal::GALILEO_E5b:
      os << std::string_view("GALILEO_E5b");
      break;
  }
  return os;
};
};  // namespace GnssSignal

namespace MeasurementType {
enum MeasurementType { UNKNOWN = 0, PSEUDORANGE = 1, DOPPLER = 2, PHASE = 4, CNO = 8 };
std::ostream& operator<<(std::ostream& os, const MeasurementType Token) {
  if (Token == 0) {
    os << std::string_view("UNKNOWN");
  } else {
    uint8_t bitmask = 0b00000001;
    std::ostringstream oss;
    while (bitmask) {
      switch (Token & bitmask) {
        case MeasurementType::PSEUDORANGE:
          oss << "PSEUDORANGE & ";
          break;
        case MeasurementType::DOPPLER:
          oss << "DOPPLER & ";
          break;
        case MeasurementType::PHASE:
          oss << "PHASE & ";
          break;
        case MeasurementType::CNO:
          oss << "CNO & ";
          break;
      }
      bitmask <<= 1;
    }
    std::string str = oss.str();
    str.erase(str.length() - 3);
    os << str;
  }
  return os;
};
};  // namespace MeasurementType

namespace ChannelState {
enum ChannelState { OFF = 0, IDLE = 1, ACQUIRING = 2, TRACKING = 3 };
std::ostream& operator<<(std::ostream& os, const ChannelState Token) {
  switch (Token) {
    case ChannelState::OFF:
      os << std::string_view("OFF");
      break;
    case ChannelState::IDLE:
      os << std::string_view("IDLE");
      break;
    case ChannelState::ACQUIRING:
      os << std::string_view("ACQUIRING");
      break;
    case ChannelState::TRACKING:
      os << std::string_view("TRACKING");
      break;
  }
  return os;
};
};  // namespace ChannelState

namespace TrackingFlags {
enum TrackingFlags {
  UNKNOWN = 0,         // 0000 0000 - No tracking
  ACQUIRED = 1,        // 0000 0001 - Signal acquired
  CODE_LOCK = 2,       // 0000 0010 - Code Locked
  CARRIER_LOCK = 4,    // 0000 0100 - Carrier Locked
  BIT_SYNC = 8,        // 0000 1000 - Data Bit found
  SUBFRAME_SYNC = 16,  // 0001 0000 - Subframe identified
  TOW_DECODED = 32,    // 0010 0000 - Subframe ToW decoded
  EPH_DECODED = 64,    // 0100 0000 - Complete ephemeris decoded (excluding atmospheric parameters)
  FINE_LOCK = 128      // 1000 0000 - Fine tracking lock
};
std::ostream& operator<<(std::ostream& os, const TrackingFlags Token) {
  if (Token == 0) {
    os << std::string_view("UNKNOWN");
  } else {
    uint8_t bitmask = 0b00000001;
    std::ostringstream oss;
    while (bitmask) {
      switch (Token & bitmask) {
        case TrackingFlags::ACQUIRED:
          oss << "ACQUIRED & ";
          break;
        case TrackingFlags::CODE_LOCK:
          oss << "CODE_LOCK & ";
          break;
        case TrackingFlags::CARRIER_LOCK:
          oss << "CARRIER_LOCK & ";
          break;
        case TrackingFlags::BIT_SYNC:
          oss << "BIT_SYNC & ";
          break;
        case TrackingFlags::SUBFRAME_SYNC:
          oss << "SUBFRAME_SYNC & ";
          break;
        case TrackingFlags::TOW_DECODED:
          oss << "TOW_DECODED & ";
          break;
        case TrackingFlags::EPH_DECODED:
          oss << "EPH_DECODED & ";
          break;
        case TrackingFlags::FINE_LOCK:
          oss << "FINE_LOCK & ";
          break;
      }
      bitmask <<= 1;
    }
    std::string str = oss.str();
    str.erase(str.length() - 3);
    os << str;
  }
  return os;
};
};  // namespace TrackingFlags

//! ------------------------------------------------------------------------------------------------

/**
 * @brief Configuration parameters provided by YAML file
 */
struct GeneralConfig {
  std::string scenario;
  std::string in_file;
  std::string out_folder;
  spdlog::level::level_enum log_level;
  uint64_t ms_to_process;
  uint16_t ms_chunk_size;
  double reference_pos_x;
  double reference_pos_y;
  double reference_pos_z;
};
struct RfSignalConfig {
  double samp_freq;
  double intmd_freq;
  bool is_complex;
  uint8_t bit_depth;
  std::vector<std::string> signals;
  std::vector<uint8_t> max_channels;
};
struct AcquisitionConfig {
  double threshold;
  double doppler_range;
  double doppler_step;
  uint8_t num_coh_per;
  uint8_t num_noncoh_per;
};
struct TrackingConfig {
  uint16_t min_concerg_time_ms;
  double tap_epl_wide;
  double tap_epl_standard;
  double tap_epl_narrow;
  double dll_bw_wide;
  double pll_bw_wide;
  double fll_bw_wide;
  double dll_bw_standard;
  double pll_bw_standard;
  double fll_bw_standard;
  double dll_bw_narrow;
  double pll_bw_narrow;
  double fll_bw_narrow;
};
struct NavigationConfig {
  bool use_psr;
  bool use_doppler;
  bool use_adr;
  bool use_cno;
  bool do_vt;
  uint8_t meas_freq;
  std::string clock_model;
  double process_std;
  double nominal_transit_time;
};
struct Config {
  GeneralConfig general;
  RfSignalConfig rfsignal;
  AcquisitionConfig acquisition;
  TrackingConfig tracking;
  NavigationConfig navigation;
};

/**
 * @brief Packet of data saved to the log file
 */
struct ChannelPacket {
  uint8_t ChannelNum{255};
  uint8_t Constellation{GnssSystem::UNKNOWN};
  uint8_t Signal{GnssSignal::UNKNOWN};
  uint8_t SVID{255};
  uint8_t ChannelStatus{ChannelState::OFF};
  uint8_t TrackingStatus{TrackingFlags::UNKNOWN};
  uint16_t Week{65535};
  double ToW{std::nan("1")};
  double CNo{std::nan("1")};
  double Doppler{std::nan("1")};
  double CodePhase{std::nan("1")};
  double CarrierPhase{std::nan("1")};
  double IE{std::nan("1")};
  double IP{std::nan("1")};
  double IL{std::nan("1")};
  double QE{std::nan("1")};
  double QP{std::nan("1")};
  double QL{std::nan("1")};
  double IP1{std::nan("1")};
  double IP2{std::nan("1")};
  double QP1{std::nan("1")};
  double QP2{std::nan("1")};
  double DllDisc{std::nan("1")};
  double PllDisc{std::nan("1")};
  double FllDisc{std::nan("1")};
};

/**
 * @brief Minimal packet of navigation data to be sent to the Navigator
 */
struct NavPacket {
  uint8_t ChannelNum{255};
  uint8_t Constellation{GnssSystem::UNKNOWN};
  uint8_t Signal{GnssSignal::UNKNOWN};
  uint8_t SVID{255};
  uint16_t Week{65535};
  double ToW{std::nan("1")};
  double CNo{std::nan("1")};
  double Doppler{std::nan("1")};
  double CodePhase{std::nan("1")};
  double CarrierPhase{std::nan("1")};
};

}  // end namespace sturdr

#endif