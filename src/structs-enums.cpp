/**
 * *structs-enums.cpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/structs-enums.cpp
 * @brief   Common structs and enum definitions used across STURDR.
 * @date    January 2025
 * @author  Daniel Sturdivant <sturdivant20@gmail.com>
 * =======  ========================================================================================
 */

#include "sturdr/structs-enums.hpp"

#include <fmt/core.h>

// #include <fmt/format.h>

#include <format>
#include <iomanip>
#include <sstream>

//! ------------------------------------------------------------------------------------------------
//! printing overrides

// *=== GnssSystem ===*
std::ostream& operator<<(std::ostream& os, const sturdr::GnssSystem::GnssSystem& Token) {
  switch (Token) {
    case sturdr::GnssSystem::GnssSystem::UNKNOWN:
      os << "UNKNOWN";
      break;
    case sturdr::GnssSystem::GnssSystem::GPS:
      os << "GPS";
      break;
    case sturdr::GnssSystem::GnssSystem::GALILEO:
      os << "GALILEO";
      break;
    case sturdr::GnssSystem::GnssSystem::GLONASS:
      os << "GLONASS";
      break;
    case sturdr::GnssSystem::GnssSystem::BEIDOU:
      os << "BEIDOU";
      break;
    case sturdr::GnssSystem::GnssSystem::QZSS:
      os << "QZSS";
      break;
    case sturdr::GnssSystem::GnssSystem::IRNSS:
      os << "IRNSS";
      break;
    case sturdr::GnssSystem::GnssSystem::SBAS:
      os << "SBAS";
      break;
  }
  return os;
}

// *=== GnssSignal ===*
std::ostream& operator<<(std::ostream& os, const sturdr::GnssSignal::GnssSignal& Token) {
  switch (Token) {
    case sturdr::GnssSignal::GnssSignal::UNKNOWN:
      os << "UNKNOWN";
      break;
    case sturdr::GnssSignal::GnssSignal::GPS_L1CA:
      os << "GPS_L1CA";
      break;
    case sturdr::GnssSignal::GnssSignal::GPS_L1C:
      os << "GPS_L1C";
      break;
    case sturdr::GnssSignal::GnssSignal::GPS_L2C:
      os << "GPS_L2C";
      break;
    case sturdr::GnssSignal::GnssSignal::GPS_L5:
      os << "GPS_L5";
      break;
    case sturdr::GnssSignal::GnssSignal::GALILEO_E1:
      os << "GALILEO_E1";
      break;
    case sturdr::GnssSignal::GnssSignal::GALILEO_E6:
      os << "GALILEO_E6";
      break;
    case sturdr::GnssSignal::GnssSignal::GALILEO_E5:
      os << "GALILEO_E5";
      break;
    case sturdr::GnssSignal::GnssSignal::GALILEO_E5a:
      os << "GALILEO_E5a";
      break;
    case sturdr::GnssSignal::GnssSignal::GALILEO_E5b:
      os << "GALILEO_E5b";
      break;
  }
  return os;
}

// *=== MeasurementType ===*
std::ostream& operator<<(std::ostream& os, const sturdr::MeasurementType::MeasurementType& Token) {
  if (Token == 0) {
    os << "UNKNOWN";
  } else {
    uint8_t bitmask = 0b00000001;
    std::ostringstream oss;
    while (bitmask) {
      switch (Token & bitmask) {
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
    os << str;
  }
  return os;
}

// *=== ChannelState ===*
std::ostream& operator<<(std::ostream& os, const sturdr::ChannelState::ChannelState& Token) {
  switch (Token) {
    case sturdr::ChannelState::ChannelState::OFF:
      os << "OFF";
      break;
    case sturdr::ChannelState::ChannelState::IDLE:
      os << "IDLE";
      break;
    case sturdr::ChannelState::ChannelState::ACQUIRING:
      os << "ACQUIRING";
      break;
    case sturdr::ChannelState::ChannelState::TRACKING:
      os << "TRACKING";
      break;
  }
  return os;
}

// *=== TrackingFlags ===*
std::ostream& operator<<(std::ostream& os, const sturdr::TrackingFlags::TrackingFlags& Token) {
  if (Token == 0) {
    os << "UNKNOWN";
  } else {
    uint8_t bitmask = 0b00000001;
    std::ostringstream oss;
    while (bitmask) {
      switch (Token & bitmask) {
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
    os << str;
  }
  return os;
}

//! ------------------------------------------------------------------------------------------------
//! these print overrides are specific to making nice csv files

// *=== ChannelPacket ===*
std::ostream& operator<<(std::ostream& os, const sturdr::ChannelPacket& c) {
  os << std::setprecision(17) << (int)c.Header.ChannelNum << ","
     << (sturdr::GnssSystem::GnssSystem)c.Header.Constellation << ","
     << (sturdr::GnssSignal::GnssSignal)c.Header.Signal << "," << (int)c.Header.SVID << ","
     << std::format("0b{:08b}", c.ChannelStatus) << "," << std::format("0b{:08b}", c.TrackingStatus)
     << "," << (int)c.Week << "," << c.ToW << "," << c.CNo << "," << c.Doppler << "," << c.CodePhase
     << "," << c.CarrierPhase << "," << c.IE << "," << c.IP << "," << c.IL << "," << c.QE << ","
     << c.QP << "," << c.QL << "," << c.IP1 << "," << c.IP2 << "," << c.QP1 << "," << c.QP2 << ","
     << c.DllDisc << "," << c.PllDisc << "," << c.FllDisc << "\n";
  return os;
}

// *=== ChannelNavPacket ===*
std::ostream& operator<<(std::ostream& os, const sturdr::ChannelNavPacket& c) {
  os << std::setprecision(17) << (int)c.Header.ChannelNum << ","
     << (sturdr::GnssSystem::GnssSystem)c.Header.Constellation << ","
     << (sturdr::GnssSignal::GnssSignal)c.Header.Signal << "," << (int)c.Header.SVID << (int)c.Week
     << "," << c.ToW << "," << c.CNo << "," << c.Doppler << "," << c.CodePhase << ","
     << c.CarrierPhase << c.DllDisc << "," << c.PllDisc << "," << c.FllDisc << "\n";
  return os;
}

// *=== ChannelEphemPacket ===*
std::ostream& operator<<(std::ostream& os, const sturdr::ChannelEphemPacket& c) {
  os << std::setprecision(17) << (int)c.Header.ChannelNum << ","
     << (sturdr::GnssSystem::GnssSystem)c.Header.Constellation << ","
     << (sturdr::GnssSignal::GnssSignal)c.Header.Signal << "," << (int)c.Header.SVID << ""
     << c.Eph.iode << "," << c.Eph.iodc << "," << c.Eph.toe << "," << c.Eph.toc << "," << c.Eph.tgd
     << "," << c.Eph.af2 << "," << c.Eph.af1 << "," << c.Eph.af0 << "," << c.Eph.e << ","
     << c.Eph.sqrtA << "," << c.Eph.deltan << "," << c.Eph.m0 << "," << c.Eph.omega0 << ","
     << c.Eph.omega << "," << c.Eph.omegaDot << "," << c.Eph.i0 << "," << c.Eph.iDot << ","
     << c.Eph.cuc << "," << c.Eph.cus << "," << c.Eph.cic << "," << c.Eph.cis << "," << c.Eph.crc
     << "," << c.Eph.crs << "," << c.Eph.ura << "," << c.Eph.health << "\n";
  return os;
}

// *=== KeplerElements ===*
std::ostream& operator<<(std::ostream& os, const satutils::KeplerElements<double>& c) {
  os << std::setprecision(17) << c.iode << "," << c.iodc << "," << c.toe << "," << c.toc << ","
     << c.tgd << "," << c.af2 << "," << c.af1 << "," << c.af0 << "," << c.e << "," << c.sqrtA << ","
     << c.deltan << "," << c.m0 << "," << c.omega0 << "," << c.omega << "," << c.omegaDot << ","
     << c.i0 << "," << c.iDot << "," << c.cuc << "," << c.cus << "," << c.cic << "," << c.cis << ","
     << c.crc << "," << c.crs << "," << c.ura << "," << c.health << "\n";
  return os;
}
