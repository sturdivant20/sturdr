/**
 * *structs-enums.hpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/structs-enums.hpp
 * @brief   Common structs and enum definitions used across STURDR.
 * @date    January 2025
 * @author  Daniel Sturdivant <sturdivant20@gmail.com>
 * =======  ========================================================================================
 */

#ifndef STURDR_STRUCTS_ENUMS_HPP
#define STURDR_STRUCTS_ENUMS_HPP

#include <spdlog/spdlog.h>

#include <cmath>
#include <cstdint>
#include <iostream>
#include <satutils/ephemeris.hpp>
#include <string>

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
};  // namespace GnssSignal

namespace MeasurementType {
enum MeasurementType { UNKNOWN = 0, PSEUDORANGE = 1, DOPPLER = 2, PHASE = 4, CNO = 8 };
};  // namespace MeasurementType

namespace ChannelState {
enum ChannelState { OFF = 0, IDLE = 1, ACQUIRING = 2, TRACKING = 4 };
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
  uint16_t ms_read_size;
  double reference_pos_x;
  double reference_pos_y;
  double reference_pos_z;
};
struct RfSignalConfig {
  double samp_freq;
  double intmd_freq;
  bool is_complex;
  uint8_t bit_depth;
  std::string signals;
  uint8_t max_channels;
};
struct AcquisitionConfig {
  double threshold;
  double doppler_range;
  double doppler_step;
  uint8_t num_coh_per;
  uint8_t num_noncoh_per;
  uint16_t max_failed_attempts;
};
struct TrackingConfig {
  uint16_t min_converg_time_ms;
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
 * @brief Minimal header for packets shared across STURDR
 */
struct HeaderPacket {
  uint8_t ChannelNum{255};
  uint8_t Constellation{GnssSystem::UNKNOWN};
  uint8_t Signal{GnssSignal::UNKNOWN};
  uint8_t SVID{255};
};

/**
 * @brief Minimal packet of data saved to the log file
 */
struct ChannelPacket {
  HeaderPacket Header;
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
 * @brief Minimal packet of ephemeris data to be sent to the navigation controller
 */
struct ChannelEphemPacket {
  HeaderPacket Header;
  satutils::KeplerElements<double> Eph;
};

/**
 * @brief Minimal packet of navigation data to be sent to the navigation controller
 */
struct ChannelNavPacket {
  HeaderPacket Header;
  uint16_t Week{65535};
  double ToW{std::nan("1")};
  double CNo{std::nan("1")};
  double Psrdot{std::nan("1")};
  double CodePhaseSec{std::nan("1")};
  double CarrierPhase{std::nan("1")};
  double DllDisc{std::nan("1")};
  double PllDisc{std::nan("1")};
  double FllDisc{std::nan("1")};
  double PsrVar{std::nan("1")};
  double PsrdotVar{std::nan("1")};
};

/**
 * @brief Data stored by navigation controller for each channel
 */
struct ChannelNavData {
  satutils::KeplerEphem<double> Sv;
  uint16_t Week{65535};
  double ToW{std::nan("1")};
  double CNo{std::nan("1")};
  double Psrdot{std::nan("1")};
  double CodePhaseSec{std::nan("1")};
  double CarrierPhase{std::nan("1")};
  double DllDisc{std::nan("1")};
  double PllDisc{std::nan("1")};
  double FllDisc{std::nan("1")};
  double PsrVar{std::nan("1")};
  double PsrdotVar{std::nan("1")};
  bool HasEphem{false};
  bool HasData{true};
};

};  // end namespace sturdr

//! ------------------------------------------------------------------------------------------------
//! printing overrides

// *=== GnssSystem ===*
std::ostream& operator<<(std::ostream& os, const sturdr::GnssSystem::GnssSystem& Token);
template <>
struct fmt::formatter<sturdr::GnssSystem::GnssSystem> : formatter<string_view> {
  auto format(sturdr::GnssSystem::GnssSystem& c, format_context& ctx) const {
    string_view name = "";
    switch (c) {
      case sturdr::GnssSystem::GnssSystem::UNKNOWN:
        name = "UNKNOWN";
        break;
      case sturdr::GnssSystem::GnssSystem::GPS:
        name = "GPS";
        break;
      case sturdr::GnssSystem::GnssSystem::GALILEO:
        name = "GALILEO";
        break;
      case sturdr::GnssSystem::GnssSystem::GLONASS:
        name = "GLONASS";
        break;
      case sturdr::GnssSystem::GnssSystem::BEIDOU:
        name = "BEIDOU";
        break;
      case sturdr::GnssSystem::GnssSystem::QZSS:
        name = "QZSS";
        break;
      case sturdr::GnssSystem::GnssSystem::IRNSS:
        name = "IRNSS";
        break;
      case sturdr::GnssSystem::GnssSystem::SBAS:
        name = "SBAS";
        break;
    }
    return formatter<string_view>::format(name, ctx);
  };
};

// *=== GnssSignal ===*
std::ostream& operator<<(std::ostream& os, const sturdr::GnssSignal::GnssSignal& Token);
template <>
struct fmt::formatter<sturdr::GnssSignal::GnssSignal> : formatter<string_view> {
  auto format(sturdr::GnssSignal::GnssSignal& c, format_context& ctx) const {
    string_view name = "";
    switch (c) {
      case sturdr::GnssSignal::GnssSignal::UNKNOWN:
        name = "UNKNOWN";
        break;
      case sturdr::GnssSignal::GnssSignal::GPS_L1CA:
        name = "GPS_L1CA";
        break;
      case sturdr::GnssSignal::GnssSignal::GPS_L1C:
        name = "GPS_L1C";
        break;
      case sturdr::GnssSignal::GnssSignal::GPS_L2C:
        name = "GPS_L2C";
        break;
      case sturdr::GnssSignal::GnssSignal::GPS_L5:
        name = "GPS_L5";
        break;
      case sturdr::GnssSignal::GnssSignal::GALILEO_E1:
        name = "GALILEO_E1";
        break;
      case sturdr::GnssSignal::GnssSignal::GALILEO_E6:
        name = "GALILEO_E6";
        break;
      case sturdr::GnssSignal::GnssSignal::GALILEO_E5:
        name = "GALILEO_E5";
        break;
      case sturdr::GnssSignal::GnssSignal::GALILEO_E5a:
        name = "GALILEO_E5a";
        break;
      case sturdr::GnssSignal::GnssSignal::GALILEO_E5b:
        name = "GALILEO_E5b";
        break;
    }
    return formatter<string_view>::format(name, ctx);
  };
};

// *=== MeasurementType ===*
std::ostream& operator<<(std::ostream& os, const sturdr::MeasurementType::MeasurementType& Token);
template <>
struct fmt::formatter<sturdr::MeasurementType::MeasurementType> : formatter<string_view> {
  auto format(sturdr::MeasurementType::MeasurementType& c, format_context& ctx) const {
    if (c == 0) {
      return formatter<string_view>::format("UNKNOWN", ctx);
    } else {
      uint8_t bitmask = 0b00000001;
      std::ostringstream oss;
      while (bitmask) {
        switch (c & bitmask) {
          case sturdr::MeasurementType::MeasurementType::PSEUDORANGE:
            oss << "PSEUDORANGE & ";
            break;
          case sturdr::MeasurementType::MeasurementType::DOPPLER:
            oss << "DOPPLER & ";
            break;
          case sturdr::MeasurementType::MeasurementType::PHASE:
            oss << "PHASE & ";
            break;
          case sturdr::MeasurementType::MeasurementType::CNO:
            oss << "CNO & ";
            break;
        }
        bitmask <<= 1;
      }
      std::string str = oss.str();
      str.erase(str.length() - 3);
      return formatter<string_view>::format(str, ctx);
    }
  };
};

// *=== ChannelState ===*
inline std::ostream& operator<<(std::ostream& os, const sturdr::ChannelState::ChannelState& Token);
template <>
struct fmt::formatter<sturdr::ChannelState::ChannelState> : formatter<string_view> {
  auto format(sturdr::ChannelState::ChannelState& c, format_context& ctx) const {
    string_view name = "";
    switch (c) {
      case sturdr::ChannelState::ChannelState::OFF:
        name = "OFF";
        break;
      case sturdr::ChannelState::ChannelState::IDLE:
        name = "IDLE";
        break;
      case sturdr::ChannelState::ChannelState::ACQUIRING:
        name = "ACQUIRING";
        break;
      case sturdr::ChannelState::ChannelState::TRACKING:
        name = "TRACKING";
        break;
    }
    return formatter<string_view>::format(name, ctx);
  };
};

// *=== TrackingFlags ===*
inline std::ostream& operator<<(
    std::ostream& os, const sturdr::TrackingFlags::TrackingFlags& Token);
template <>
struct fmt::formatter<sturdr::TrackingFlags::TrackingFlags> : formatter<string_view> {
  auto format(sturdr::TrackingFlags::TrackingFlags& c, format_context& ctx) const {
    if (c == 0) {
      return formatter<string_view>::format("UNKNOWN", ctx);
    } else {
      uint8_t bitmask = 0b00000001;
      std::ostringstream oss;
      while (bitmask) {
        switch (c & bitmask) {
          case sturdr::TrackingFlags::TrackingFlags::ACQUIRED:
            oss << "ACQUIRED & ";
            break;
          case sturdr::TrackingFlags::TrackingFlags::CODE_LOCK:
            oss << "CODE_LOCK & ";
            break;
          case sturdr::TrackingFlags::TrackingFlags::CARRIER_LOCK:
            oss << "CARRIER_LOCK & ";
            break;
          case sturdr::TrackingFlags::TrackingFlags::BIT_SYNC:
            oss << "BIT_SYNC & ";
            break;
          case sturdr::TrackingFlags::TrackingFlags::SUBFRAME_SYNC:
            oss << "SUBFRAME_SYNC & ";
            break;
          case sturdr::TrackingFlags::TrackingFlags::TOW_DECODED:
            oss << "TOW_DECODED & ";
            break;
          case sturdr::TrackingFlags::TrackingFlags::EPH_DECODED:
            oss << "EPH_DECODED & ";
            break;
          case sturdr::TrackingFlags::TrackingFlags::FINE_LOCK:
            oss << "FINE_LOCK & ";
            break;
        }
        bitmask <<= 1;
      }
      std::string str = oss.str();
      str.erase(str.length() - 3);
      return formatter<string_view>::format(str, ctx);
    }
  };
};

//! ------------------------------------------------------------------------------------------------
//! these print overrides are specific to making nice csv files

// *=== ChannelPacket ===*
std::ostream& operator<<(std::ostream& os, const sturdr::ChannelPacket& c);
template <>
struct fmt::formatter<sturdr::ChannelPacket> : formatter<string_view> {
  auto format(sturdr::ChannelPacket& c, format_context& ctx) const {
    std::ostringstream oss;
    oss << std::setprecision(17) << (int)c.Header.ChannelNum << ","
        << (sturdr::GnssSystem::GnssSystem)c.Header.Constellation << ","
        << (sturdr::GnssSignal::GnssSignal)c.Header.Signal << "," << (int)c.Header.SVID << ","
        << std::format("0b{:08b}", c.ChannelStatus) << ","
        << std::format("0b{:08b}", c.TrackingStatus) << "," << (int)c.Week << "," << c.ToW << ","
        << c.CNo << "," << c.Doppler << "," << c.CodePhase << "," << c.CarrierPhase << "," << c.IE
        << "," << c.IP << "," << c.IL << "," << c.QE << "," << c.QP << "," << c.QL << "," << c.IP1
        << "," << c.IP2 << "," << c.QP1 << "," << c.QP2 << "," << c.DllDisc << "," << c.PllDisc
        << "," << c.FllDisc;
    return formatter<string_view>::format(oss.str(), ctx);
  };
};

// *=== ChannelNavPacket ===*
std::ostream& operator<<(std::ostream& os, const sturdr::ChannelNavPacket& c);
template <>
struct fmt::formatter<sturdr::ChannelNavPacket> : formatter<string_view> {
  auto format(sturdr::ChannelNavPacket& c, format_context& ctx) const {
    std::ostringstream oss;
    oss << std::setprecision(17) << (int)c.Header.ChannelNum << ","
        << (sturdr::GnssSystem::GnssSystem)c.Header.Constellation << ","
        << (sturdr::GnssSignal::GnssSignal)c.Header.Signal << "," << (int)c.Header.SVID
        << (int)c.Week << "," << c.ToW << "," << c.CNo << "," << c.Psrdot << "," << c.CodePhaseSec
        << "," << c.CarrierPhase << c.DllDisc << "," << c.PllDisc << "," << c.FllDisc;
    return formatter<string_view>::format(oss.str(), ctx);
  };
};

// *=== ChannelEphemPacket ===*
std::ostream& operator<<(std::ostream& os, const sturdr::ChannelEphemPacket& c);
template <>
struct fmt::formatter<sturdr::ChannelEphemPacket> : formatter<string_view> {
  auto format(sturdr::ChannelEphemPacket& c, format_context& ctx) const {
    std::ostringstream oss;
    oss << std::setprecision(17) << (int)c.Header.ChannelNum << ","
        << (sturdr::GnssSystem::GnssSystem)c.Header.Constellation << ","
        << (sturdr::GnssSignal::GnssSignal)c.Header.Signal << "," << (int)c.Header.SVID << ""
        << c.Eph.iode << "," << c.Eph.iodc << "," << c.Eph.toe << "," << c.Eph.toc << ","
        << c.Eph.tgd << "," << c.Eph.af2 << "," << c.Eph.af1 << "," << c.Eph.af0 << "," << c.Eph.e
        << "," << c.Eph.sqrtA << "," << c.Eph.deltan << "," << c.Eph.m0 << "," << c.Eph.omega0
        << "," << c.Eph.omega << "," << c.Eph.omegaDot << "," << c.Eph.i0 << "," << c.Eph.iDot
        << "," << c.Eph.cuc << "," << c.Eph.cus << "," << c.Eph.cic << "," << c.Eph.cis << ","
        << c.Eph.crc << "," << c.Eph.crs << "," << c.Eph.ura << "," << c.Eph.health;
    return formatter<string_view>::format(oss.str(), ctx);
  };
};

// *=== KeplerElements ===*
std::ostream& operator<<(std::ostream& os, const satutils::KeplerElements<double>& c);
template <>
struct fmt::formatter<satutils::KeplerElements<double>> : formatter<string_view> {
  auto format(satutils::KeplerElements<double>& c, format_context& ctx) const {
    std::ostringstream oss;
    oss << std::setprecision(17) << c.iode << "," << c.iodc << "," << c.toe << "," << c.toc << ","
        << c.tgd << "," << c.af2 << "," << c.af1 << "," << c.af0 << "," << c.e << "," << c.sqrtA
        << "," << c.deltan << "," << c.m0 << "," << c.omega0 << "," << c.omega << "," << c.omegaDot
        << "," << c.i0 << "," << c.iDot << "," << c.cuc << "," << c.cus << "," << c.cic << ","
        << c.cis << "," << c.crc << "," << c.crs << "," << c.ura << "," << c.health;
    return formatter<string_view>::format(oss.str(), ctx);
  };
};

#endif