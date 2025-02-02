/**
 * *discriminator.hpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/discriminator.hpp
 * @brief   Standard satellite tracking match filter discriminators.
 * @date    January 2025
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

#ifndef STURDR_DISCRIMINATOR_HPP
#define STURDR_DISCRIMINATOR_HPP

#include <complex>

namespace sturdr {

//! ----- DLL --------------------------------------------------------------------------------------

/**
 * *=== DllNneml ===*
 * @brief Delay Lock Loop - Normalized non-coherent early minus late discriminator
 * @param E   Early discriminator
 * @param L   Late discriminator
 * @return chip error/misalignment [chip]
 */
double DllNneml(const std::complex<double> &E, const std::complex<double> &L);
double DllNneml(const double &IE, const double &QE, const double &IL, const double &QL);

/**
 * *=== DllNneml2 ===*
 * @brief Delay Lock Loop - Normalized non-coherent early minus late discriminator squared
 * @param E   Early discriminator
 * @param L   Late discriminator
 * @return chip error/misalignment [chip]
 */
double DllNneml2(const std::complex<double> &E, const std::complex<double> &L);
double DllNneml2(const double &IE, const double &QE, const double &IL, const double &QL);

/**
 * *=== DllNcdp ===*
 * @brief Delay Lock Loop - Normalized coherent dot product discriminator
 * @param E   Early discriminator
 * @param P   Prompt discriminator
 * @param L   Late discriminator
 * @return chip error/misalignment [chip]
 */
double DllNcdp(
    const std::complex<double> &E, const std::complex<double> &P, const std::complex<double> &L);
double DllNcdp(const double &IE, const double &IP, const double &IL);

/**
 * *=== DllVariance ===
 * @brief Variance in the DLL discriminator
 * @param cno   Carrier to noise density ratio magnitude (not dB-Hz)
 * @param T     Integration time [s]
 * @return variance in the DLL discriminator [chips^2]
 */
double DllVariance(const double &cno, const double &T);

//! ----- PLL --------------------------------------------------------------------------------------

/**
 * *=== PllCostas ===
 * @brief Phase Lock Loop - Costas discriminator
 * @param P   Prompt correlator
 * @return Phase error/misalignment [rad]
 */
double PllCostas(const std::complex<double> &P);
double PllCostas(const double &IP, const double &QP);

/**
 * *=== PllAtan2 ===
 * @brief Phase Lock Loop - ATAN2 discriminator, sensitive to data bits
 * @param P   Prompt correlator
 * @return Phase error/misalignment [rad]
 */
double PllAtan2(const std::complex<double> &P);
double PllAtan2(const double &IP, const double &QP);

/**
 * *=== PllNddc ===*
 * @brief Phase Lock Loop - Normalized decision-directed-costas discriminator
 * @param P   Prompt correlator
 * @return Phase error/misalignment [rad]
 */
double PllNddc(const std::complex<double> &P);
double PllNddc(const double &IP, const double &QP);

/**
 * *=== PllVariance ===*
 * @param cno   Carrier to noise density ratio magnitude (not dB-Hz)
 * @param T     Integration time [s]
 * @return variance in the PLL discriminator [rad^2]
 */
double PllVariance(const double &cno, const double &T);

//! ----- FLL --------------------------------------------------------------------------------------

/**
 * *=== FllAtan2 ===
 * @brief Frequency Lock Loop - ATAN2 discriminator
 * @param P1  First half prompt correlator
 * @param P2  Second half prompt correlator
 * @param T   Integration time of half-correlator [s]
 * @return frequency error/misalignment [rad/s]
 */
double FllAtan2(const std::complex<double> &P1, const std::complex<double> &P2, const double &T);
double FllAtan2(
    const double &IP1, const double &QP1, const double &IP2, const double &QP2, const double &T);

/**
 * *=== FllNddcp ===
 * @brief Frequency Lock Loop - Normalized decision-directed-cross-product discriminator
 * @param P1  First half prompt correlator
 * @param P2  Second half prompt correlator
 * @param T   Integration time of half-correlator [s]
 * @return frequency error/misalignment [rad/s]
 */
double FllNddcp(
    const std::complex<double> &P1,
    const std::complex<double> &P2,
    const std::complex<double> &P,
    const double &T);
double FllNddcp(
    const double &IP1,
    const double &QP1,
    const double &IP2,
    const double &QP2,
    const double &IP,
    const double &QP,
    const double &T);

/**
 * *=== FllVariance ===*
 * @param cno   Carrier to noise density ratio magnitude (not dB-Hz)
 * @param T     Integration time [s]
 * @return variance in the FLL discriminator [(rad/s)^2]
 */
double FllVariance(const double &cno, const double &T);

}  // end namespace sturdr

#endif