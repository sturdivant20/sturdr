/**
 * *tracking.hpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/dsp/tracking.hpp
 * @brief   Satellite scalar tracking methods.
 * @date    December 2024
 * @author  Daniel Sturdivant <sturdivant20@gmail.com>
 * @ref     1. "Understanding GPS/GNSS Principles and Applications", 3rd Edition, 2017
 *              - Kaplan & Hegarty
 *          2. "Are PLLs Dead? A Tutorial on Kalman Filter-Based Techniques for Digital Carrier
 *              Syncronization" - Vila-Valls, Closas, Navarro, Fernandez-Prades
 *          3. "A Software-Defined GPS and Galileo Receiver: A Single-Frequency Approach", 2007
 *              - Borre, Akos, Bertelsen, Rinder, Jensen
 *          4. "Position, Navigation, and Timing Technologies in the 21st Century", Volume 1, 2021
 *              - Morton, Diggelen, Spilker Jr., Parkinson
 * =======  ========================================================================================
 */

#ifndef STURDR_TRACKING_HPP
#define STURDR_TRACKING_HPP

#include <Eigen/Dense>

namespace sturdr {

/**
 * *=== NaturalFrequency ===*
 * @brief Calculate the natural radian frequency of the loop filter given the noise bandwidth
 * @param bw    Noise bandwidth [Hz]
 * @param order Loop filter order (1, 2, or 3)
 * @return Loop filter's natural radian frequency [rad/s]
 */
double NaturalFrequency(const double &bw, const int order);

/**
 * *=== FLLassistedPLL_2ndOrder ===*
 * @brief 2nd order PLL/DLL assisted by 1st order FLL Digital Loop Filter
 * @param nco_freq  Doppler/Frequency estimate from the Digital Loop Filter
 * @param vel_accum Velocity accumulator memory
 * @param phase_err Phase error input (discriminator) [rad]
 * @param freq_err  Frequency error input (discriminator) [rad/s]
 * @param T         Coherent integration time [s]
 * @param w0p       Natural radian frequency of the PLL [rad/s]
 * @param w0f       Natural radian frequency of the FLL [rad/s]
 */
void FLLassistedPLL_2ndOrder(
    double &nco_freq,
    double &vel_accum,
    const double &phase_err,
    const double &freq_err,
    const double &T,
    const double &w0p,
    const double &w0f);

/**
 * *=== FLLassistedPLL_3rdOrder ===*
 * @brief 3rd order PLL/DLL assisted by 2nd order FLL Digital Loop Filter
 * @param nco_freq  Doppler/Frequency estimate from the Digital Loop Filter
 * @param vel_accum Velocity accumulator memory
 * @param acc_accum Acceleration accumulator memory
 * @param phase_err Phase error input (discriminator) [rad]
 * @param freq_err  Frequency error input (discriminator) [rad/s]
 * @param T         Coherent integration time [s]
 * @param w0p       Natural radian frequency of the PLL [rad/s]
 * @param w0f       Natural radian frequency of the FLL [rad/s]
 */
void FLLassistedPLL_3rdOrder(
    double &nco_freq,
    double &vel_accum,
    double &acc_accum,
    const double &phase_err,
    const double &freq_err,
    const double &T,
    const double &w0p,
    const double &w0f);

/**
 * *=== PLL_2ndOrder ===*
 * @brief 2nd order PLL/DLL
 * @param nco_freq  Doppler/Frequency estimate from the Digital Loop Filter
 * @param vel_accum Velocity accumulator memory
 * @param phase_err Phase error input (discriminator) [rad]
 * @param T         Coherent integration time [s]
 * @param w0        Natural radian frequency of the PLL/DLL [rad/s]
 */
void PLL_2ndOrder(
    double &nco_freq,
    double &vel_accum,
    const double &phase_err,
    const double &T,
    const double &w0);

/**
 * *=== PLL_3rdOrder ===*
 * @brief 3rd order PLL/DLL
 * @param nco_freq  Doppler/Frequency estimate from the Digital Loop Filter
 * @param vel_accum Velocity accumulator memory
 * @param acc_accum Acceleration accumulator memory
 * @param phase_err Phase error input (discriminator) [rad]
 * @param T         Coherent integration time [s]
 * @param w0        Natural radian frequency of the PLL/DLL [rad/s]
 */
void PLL_3rdOrder(
    double &nco_freq,
    double &vel_accum,
    double &acc_accum,
    const double &phase_err,
    const double &T,
    const double &w0);

//! ------------------------------------------------------------------------------------------------

class TrackingKF {
 public:
  /**
   * *=== TrackingKF ===*
   * @brief Constructor
   */
  TrackingKF();

  /**
   * *=== ~TrackingKF ===*
   * @brief Destructor
   */
  ~TrackingKF();

  /**
   * *=== Init ===*
   * @brief Init function
   * @param init_w0d          Natural radian frequency of the DLL [rad/s]
   * @param init_w0p          Natural radian frequency of the PLL [rad/s]
   * @param init_w0f          Natural radian frequency of the FLL [rad/s]
   * @param init_carr_phase   Initial carrier phase estimate [rad]
   * @param init_carr_doppler Initial doppler estimate [rad/s]
   * @param init_code_phase   Initial code phase estimate [chips]
   * @param init_k            Code to carrier frequency ratio [rad/chip]
   * @param init_cno          Carrier-to-noise density ratio magnitude (not dB-Hz)
   * @param init_T            Integration time [s]
   * @param intmd_freq        Intermediate frequency of the recorded signal [rad/s]
   * @param code_freq         Chipping rate of the true signal [chip/s]
   */
  void Init(
      const double &init_w0d,
      const double &init_w0p,
      const double &init_w0f,
      const double &init_k,
      const double &init_carr_phase,
      const double &init_carr_doppler,
      const double &init_code_phase,
      const double &init_cno,
      const double &init_T,
      const double &intmd_freq,
      const double &code_freq);

  /**
   * *=== UpdateDynamicsParam ===*
   * @brief Change the dynamics update parameters
   * @param w0d Natural radian frequency of the DLL [rad/s]
   * @param w0p Natural radian frequency of the PLL [rad/s]
   * @param w0f Natural radian frequency of the FLL [rad/s]
   * @param k   Code to carrier frequency ratio [rad/chip]
   * @param T   Integration time [s]
   */
  void UpdateDynamicsParam(
      const double &w0d, const double &w0p, const double &w0f, const double &k, const double &T);

  /**
   * *=== UpdateDynamicsParam ===*
   * @brief Change the dynamics update parameters
   * @param cno Carrier-to-noise density ratio magnitude (not dB-Hz)
   * @param T   Integration time [s]
   */
  void UpdateMeasurementsParam(const double &cno, const double &T);

  /**
   * *=== Run ===*
   * @brief Run the Kalman Filter DLL/PLL
   * @param chip_err  Chip discriminator [chips]
   * @param phase_err Phase discriminator [rad]
   * @param freq_err  Frequency discriminator [rad/s]
   * @return Current state estimates of PLL/DLL Kalman Filter
   *          -> (Phase, doppler, jitter, chips, chip_rate)
   */
  Eigen::Vector<double, 5> Run(double &chip_err, double &phase_err, double &freq_err);

  void SetRemCarrierPhase(const double &rem_carrier_phase);
  void SetRemCodePhase(const double &rem_code_phase);

 private:
  Eigen::Vector<double, 5> x_;
  Eigen::Matrix<double, 5, 5> P_;
  Eigen::Matrix<double, 5, 5> Q_;
  Eigen::Matrix<double, 3, 3> R_;
  Eigen::Matrix<double, 5, 5> F_;
  Eigen::Matrix<double, 5, 2> G_;
  Eigen::Matrix<double, 3, 5> H_;
  Eigen::Vector<double, 2> u_;

  void make_F(const double &k, const double &T);
  void make_G(const double &T);
  void make_Q(
      const double &w0d, const double &w0p, const double &w0f, const double &k, const double &T);
  void make_H();
  void make_R(const double &cno, const double &T);

};  // end class TrackingKF

}  // namespace sturdr
#endif