/**
 * *discriminator.cpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/discriminator.cpp
 * @brief   Standard satellite tracking match filter discriminators.
 * @date    December 2024
 * @author  Daniel Sturdivant <sturdivant20@gmail.com>
 * @ref     1. "Understanding GPS/GNSS Principles and Applications", 3rd Edition, 2017
 *              - Kaplan & Hegarty
 *          2. "Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems", 2nd
 *              Edition, 2013 - Groves
 *          3. "Global Positioning System: Signals, Measurements, and Performance", 2nd Edition,
 *              2006 - Misra & Enge
 *          4. "Position, Navigation, and Timing Technologies in the 21st Century", Volume 1, 2021
 *              - Morton, Diggelen, Spilker Jr., Parkinson
 * =======  ========================================================================================
 */

#include "sturdr/discriminator.hpp"

#include <spdlog/spdlog.h>

#include <cmath>
#include <exception>
#include <navtools/constants.hpp>

namespace sturdr {

//! ----- DLL --------------------------------------------------------------------------------------

// *=== DllNneml ===*
double DllNneml(const std::complex<double> &E, const std::complex<double> &L) {
  try {
    double Emag = std::abs(E);
    double Lmag = std::abs(L);
    return (0.5 * (Emag - Lmag) / (Emag + Lmag));
  } catch (std::exception &e) {
    spdlog::get("sturdr-console")
        ->error("discriminator.cpp DllNneml failed! Error -> {}", e.what());
    return std::nan("1");
  }
}
double DllNneml(const double &IE, const double &QE, const double &IL, const double &QL) {
  try {
    double Emag = std::sqrt(IE * IE + QE * QE);
    double Lmag = std::sqrt(IL * IL + QL * QL);
    return (0.5 * (Emag - Lmag) / (Emag + Lmag));
  } catch (std::exception &e) {
    spdlog::get("sturdr-console")
        ->error("discriminator.cpp DllNneml failed! Error -> {}", e.what());
    return std::nan("1");
  }
}

// *=== DllNneml2 ===*
double DllNneml2(const std::complex<double> &E, const std::complex<double> &L) {
  try {
    double Emag = std::norm(E);
    double Lmag = std::norm(L);
    return (0.5 * (Emag - Lmag) / (Emag + Lmag));
  } catch (std::exception &e) {
    spdlog::get("sturdr-console")
        ->error("discriminator.cpp DllNneml2 failed! Error -> {}", e.what());
    return std::nan("1");
  }
}
double DllNneml2(const double &IE, const double &QE, const double &IL, const double &QL) {
  try {
    double Emag = IE * IE + QE * QE;
    double Lmag = IL * IL + QL * QL;
    return (0.5 * (Emag - Lmag) / (Emag + Lmag));
  } catch (std::exception &e) {
    spdlog::get("sturdr-console")
        ->error("discriminator.cpp DllNneml2 failed! Error -> {}", e.what());
    return std::nan("1");
  }
}

// *=== DllNcdp ===*
double DllNcdp(
    const std::complex<double> &E, const std::complex<double> &P, const std::complex<double> &L) {
  try {
    return ((std::abs(E.real()) - std::abs(L.real())) / std::abs(P.real()));
  } catch (std::exception &e) {
    spdlog::get("sturdr-console")->error("discriminator.cpp DllNcdp failed! Error -> {}", e.what());
    return std::nan("1");
  }
}
double DllNcdp(const double &IE, const double &IP, const double &IL) {
  try {
    return ((std::abs(IE) - std::abs(IL)) / std::abs(IP));
  } catch (std::exception &e) {
    spdlog::get("sturdr-console")->error("discriminator.cpp DllNcdp failed! Error -> {}", e.what());
    return std::nan("1");
  }
}

// *=== DllVariance ===*
double DllVariance(const double &cno, const double &T) {
  try {
    double tmp = 1.0 / (cno * T);
    return (0.5 * tmp * (0.5 + tmp));
  } catch (std::exception &e) {
    spdlog::get("sturdr-console")
        ->error("discriminator.cpp DllVariance failed! Error -> {}", e.what());
    return std::nan("1");
  }
}

//! ----- PLL --------------------------------------------------------------------------------------

// *=== PllCostas ===*
double PllCostas(const std::complex<double> &P) {
  try {
    return std::atan(P.imag() / P.real());
  } catch (std::exception &e) {
    spdlog::get("sturdr-console")
        ->error("discriminator.cpp PllCostas failed! Error -> {}", e.what());
    return std::nan("1");
  }
}
double PllCostas(const double &IP, const double &QP) {
  try {
    return std::atan(QP / IP);
  } catch (std::exception &e) {
    spdlog::get("sturdr-console")
        ->error("discriminator.cpp PllCostas failed! Error -> {}", e.what());
    return std::nan("1");
  }
}

// *=== PllAtan2 ===*
double PllAtan2(const std::complex<double> &P) {
  try {
    return std::atan2(P.imag(), P.real());
  } catch (std::exception &e) {
    spdlog::get("sturdr-console")
        ->error("discriminator.cpp PllCostas failed! Error -> {}", e.what());
    return std::nan("1");
  }
}
double PllAtan2(const double &IP, const double &QP) {
  try {
    return std::atan2(QP, IP);
  } catch (std::exception &e) {
    spdlog::get("sturdr-console")
        ->error("discriminator.cpp PllAtan2 failed! Error -> {}", e.what());
    return std::nan("1");
  }
}

// *=== PllNddc ===*
double PllNddc(const std::complex<double> &P) {
  try {
    return P.imag() * std::copysign(1.0, P.real()) / std::abs(P);
  } catch (std::exception &e) {
    spdlog::get("sturdr-console")->error("discriminator.cpp PllNddc failed! Error -> {}", e.what());
    return std::nan("1");
  }
}
double PllNddc(const double &IP, const double &QP) {
  try {
    return QP * std::copysign(1.0, IP) / std::sqrt(IP * IP + QP * QP);
  } catch (std::exception &e) {
    spdlog::get("sturdr-console")->error("discriminator.cpp PllNddc failed! Error -> {}", e.what());
    return std::nan("1");
  }
}

// *=== PllVariance ===
double PllVariance(const double &cno, const double &T) {
  try {
    double tmp = 1.0 / (cno * T);
    return (tmp * (1.0 + 0.5 * tmp));
  } catch (std::exception &e) {
    spdlog::get("sturdr-console")
        ->error("discriminator.cpp PllVariance failed! Error -> {}", e.what());
    return std::nan("1");
  }
}

//! ----- FLL --------------------------------------------------------------------------------------

// *=== FllAtan2 ===*
double FllAtan2(const std::complex<double> &P1, const std::complex<double> &P2, const double &T) {
  try {
    double x = P1.real() * P2.imag() - P2.real() * P1.imag();
    double d = P1.real() * P2.real() + P1.imag() * P2.imag();
    return (std::atan2(x, d) / (0.5 * T));
  } catch (std::exception &e) {
    spdlog::get("sturdr-console")
        ->error("discriminator.cpp FllAtan2 failed! Error -> {}", e.what());
    return std::nan("1");
  }
}
double FllAtan2(
    const double &IP1, const double &QP1, const double &IP2, const double &QP2, const double &T) {
  try {
    double x = IP1 * QP2 - IP2 * QP1;
    double d = IP1 * IP2 + QP1 * QP2;
    return (std::atan2(x, d) / T);
  } catch (std::exception &e) {
    spdlog::get("sturdr-console")
        ->error("discriminator.cpp FllAtan2 failed! Error -> {}", e.what());
    return std::nan("1");
  }
}

// *=== FllNddcp ===*
double FllNddcp(
    const std::complex<double> &P1,
    const std::complex<double> &P2,
    const std::complex<double> &P,
    const double &T) {
  try {
    double x = P1.real() * P2.imag() - P2.real() * P1.imag();
    double d = P1.real() * P2.real() + P1.imag() * P2.imag();
    return (x * std::copysign(1.0, d) / (std::abs(P) * T));
  } catch (std::exception &e) {
    spdlog::get("sturdr-console")
        ->error("discriminator.cpp FllNddcp failed! Error -> {}", e.what());
    return std::nan("1");
  }
}
double FllNddcp(
    const double &IP1,
    const double &QP1,
    const double &IP2,
    const double &QP2,
    const double &IP,
    const double &QP,
    const double &T) {
  try {
    double x = IP1 * QP2 - IP2 * QP1;
    double d = IP1 * IP2 + QP1 * QP2;
    return (x * std::copysign(1.0, d) / (std::sqrt(IP * IP + QP * QP) * T));
  } catch (std::exception &e) {
    spdlog::get("sturdr-console")
        ->error("discriminator.cpp FllNddcp failed! Error -> {}", e.what());
    return std::nan("1");
  }
}

// *=== FllVariance ===
double FllVariance(const double &cno, const double &T) {
  try {
    double tmp = 1.0 / (cno * T);
    return 8.0 / (T * T) * tmp * (tmp + 1.0);
    // return (8.0 * tmp * (1.0 + tmp) / (T * T));
  } catch (std::exception &e) {
    spdlog::get("sturdr-console")
        ->error("discriminator.cpp FllVariance failed! Error -> {}", e.what());
    return std::nan("1");
  }
}

}  // end namespace sturdr