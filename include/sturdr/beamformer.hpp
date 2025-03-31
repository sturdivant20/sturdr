/**
 * *beamsteer.hpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/channel.hpp
 * @brief   Simple, deterministic beamsteering for GNSS phased array antenna
 * @date    January 2025
 * @ref     1. "Error Analysis of Carrier Phase Positioning Using Controlled Reception Pattern
 *              Antenna Arrays" (Master's thesis, Auburn University) - Josh Starling
 *          2. "A Multi-Antenna Vector Tracking Beamsteering GPS Receiver for Robust Positioning"
 *              (Master's Thesis, Auburn University) - Scott Burchfield
 * =======  ========================================================================================
 */

#ifndef STURDR_BEAMSTEER_HPP
#define STURDR_BEAMSTEER_HPP

#include <Eigen/Dense>
#include <complex>

namespace sturdr {

class BeamFormer {
 private:
  int N_;                 // number of antennas | length of weighting vector
  double lambda_;         // wavelength of carrier wave
  Eigen::Matrix3Xd xyz_;  // 3xN_ antenna body frame coordinates
  Eigen::VectorXcd w_;    // weighting vector

 public:
  /**
   * *=== BeamFormer ===*
   * @brief Constructor
   * @param n_ant   number of antennas
   * @param lambda  wavelength of carrier signal [m/rad]
   */
  BeamFormer(int n_ant, double lambda, Eigen::Matrix3Xd ant_xyz);

  /**
   * *=== ~BeamFormer ===*
   * @brief Destructor
   */
  ~BeamFormer();

  /**
   * *=== GetWeights ===*
   * @brief Grab the current weights of the beamformer
   * @returns current weighting vector
   */
  Eigen::VectorXcd GetWeights();

  /**
   * *=== CalcSteeringWeights ===*
   * @brief Calculates deterministic beam steering weights
   * @param u_body  unit vector to beam steer towards in the body frame
   */
  void CalcSteeringWeights(const Eigen::Ref<const Eigen::Vector3d>& u_body);

  /**
   * *=== CalcNullingWeights ===*
   * @brief Calculates deterministic null steering weights
   * @param u_body  unit vector to null steer towards in the body frame
   */
  void CalcNullingWeights(const Eigen::Ref<const Eigen::Vector3d>& u_body);

  /**
   * *=== () ===*
   * @brief The beamforming operation (combines x elements into 1 beamformed element)
   * @param x vector of complex elements to beamform together
   * @returns beamformed combination of x
   */
  std::complex<double> operator()(const Eigen::Ref<const Eigen::VectorXcd>& x);
};

// /**
//  * *=== DeterministicBeam ===*
//  * @brief Accumulates 'n_samp' samples of the current integration period and beamsteers toward
//  *        provided unit vector
//  * @param ant_xyz         Known antenna positions in the body frame [in cycles
//  (2*pi/lambda*xyz_m)]
//  * @param u               Desired unit vector in the body frame
//  * @param rfdata          Recorded signal data
//  * @param code            Local code to upsample
//  * @param rem_code_phase  Initial fractional phase of the code
//  * @param code_freq       GNSS signal code frequency [Hz]
//  * @param rem_carr_phase  Initial fractional phase of the carrier [rad]
//  * @param carr_freq       Current carrier frequency (including intermediate frequency) [rad/s]
//  * @param carr_jit        Current carrier frequency jitter [rad/s^2]
//  * @param samp_freq       GNSS receiver front end sampling frequency [Hz]
//  * @param half_samp       Number of samples in half the TOTAL accumulation period
//  * @param samp_remaining  Number of samples remaining to be accumulated inside TOTAL period
//  * @param t_space         Spacing between correlator taps
//  * @param E               Early correlator
//  * @param P1              Prompt first-half correlator
//  * @param P2              Prompt second-half correlator
//  * @param L               Late correlator
//  */
// void DeterministicBeam(
//     const Eigen::Ref<const Eigen::MatrixXd> &ant_xyz,
//     const Eigen::Ref<const Eigen::Vector3d> &u,
//     const Eigen::Ref<const Eigen::MatrixXcd> &rfdata,
//     const bool code[1023],
//     double &rem_code_phase,
//     double &code_freq,
//     double &rem_carr_phase,
//     double &carr_freq,
//     double &carr_jit,
//     double &samp_freq,
//     uint64_t &half_samp,
//     uint64_t &samp_remaining,
//     double &t_space,
//     std::complex<double> &E,
//     std::complex<double> &P1,
//     std::complex<double> &P2,
//     std::complex<double> &L);

// /**
//  * *=== DeterministicNull ===*
//  * @brief Accumulates 'n_samp' samples of the current integration period and nullsteers toward
//  *        provided uint vector
//  * @param ant_xyz         Known antenna positions in the body frame [in cycles
//  (2*pi/lambda*xyz_m)]
//  * @param u               Desired unit vector in the body frame
//  * @param rfdata          Recorded signal data
//  * @param code            Local code to upsample
//  * @param rem_code_phase  Initial fractional phase of the code
//  * @param code_freq       GNSS signal code frequency [Hz]
//  * @param rem_carr_phase  Initial fractional phase of the carrier [rad]
//  * @param carr_freq       Current carrier frequency (including intermediate frequency) [rad/s]
//  * @param carr_jit        Current carrier frequency jitter [rad/s^2]
//  * @param samp_freq       GNSS receiver front end sampling frequency [Hz]
//  * @param half_samp       Number of samples in half the TOTAL accumulation period
//  * @param samp_remaining  Number of samples remaining to be accumulated inside TOTAL period
//  * @param t_space         Spacing between correlator taps
//  * @param E               Early correlator
//  * @param P1              Prompt first-half correlator
//  * @param P2              Prompt second-half correlator
//  * @param L               Late correlator
//  */
// void DeterministicNull(
//     const Eigen::Ref<const Eigen::MatrixXd> &ant_xyz,
//     const Eigen::Ref<const Eigen::Vector3d> &u,
//     const Eigen::Ref<const Eigen::MatrixXcd> &rfdata,
//     const bool code[1023],
//     double &rem_code_phase,
//     double &code_freq,
//     double &rem_carr_phase,
//     double &carr_freq,
//     double &carr_jit,
//     double &samp_freq,
//     uint64_t &half_samp,
//     uint64_t &samp_remaining,
//     double &t_space,
//     std::complex<double> &E,
//     std::complex<double> &P1,
//     std::complex<double> &P2,
//     std::complex<double> &L);

// /**
//  * *=== LmsBeam ===*
//  * @brief Accumulates 'n_samp' samples of the current integration period and beamsteers using the
//  *        Least Mean Squares Algorithm
//  * @param mu              Damping
//  * @param W               Previously estimated beamsteering weights
//  * @param u               Desired unit vector in the body frame
//  * @param rfdata          Recorded signal data
//  * @param code            Local code to upsample
//  * @param rem_code_phase  Initial fractional phase of the code
//  * @param code_freq       GNSS signal code frequency [Hz]
//  * @param rem_carr_phase  Initial fractional phase of the carrier [rad]
//  * @param carr_freq       Current carrier frequency (including intermediate frequency) [rad/s]
//  * @param carr_jit        Current carrier frequency jitter [rad/s^2]
//  * @param samp_freq       GNSS receiver front end sampling frequency [Hz]
//  * @param half_samp       Number of samples in half the TOTAL accumulation period
//  * @param samp_remaining  Number of samples remaining to be accumulated inside TOTAL period
//  * @param t_space         Spacing between correlator taps
//  * @param E               Early correlator
//  * @param P1              Prompt first-half correlator
//  * @param P2              Prompt second-half correlator
//  * @param L               Late correlator
//  */
// void LmsBeam(
//     const double &mu,
//     Eigen::Ref<Eigen::VectorXcd> W,
//     Eigen::Ref<Eigen::VectorXcd> delta_W,
//     const Eigen::Ref<const Eigen::MatrixXcd> &rfdata,
//     const bool code[1023],
//     double &rem_code_phase,
//     double &code_freq,
//     double &rem_carr_phase,
//     double &carr_freq,
//     double &carr_jit,
//     double &samp_freq,
//     uint64_t &half_samp,
//     uint64_t &samp_remaining,
//     double &t_space,
//     std::complex<double> &E,
//     std::complex<double> &P1,
//     std::complex<double> &P2,
//     std::complex<double> &L);

// void LmsNormalize(Eigen::Ref<Eigen::VectorXcd> W);

}  // namespace sturdr

#endif