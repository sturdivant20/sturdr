/**
 * *beamsteer.hpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/channel.hpp
 * @brief   Simple, deterministic beamsteering for GNSS phased array antenna
 * @date    January 2025
 * @ref     1. "Error Analysis of Carrier Phase Positioning Using Controlled Reception Pattern
 *              Antenna Arrays" (Master's thesis, Auburn University) - Josh Starling
 * =======  ========================================================================================
 */

#ifndef STURDR_BEAMSTEER_HPP
#define STURDR_BEAMSTEER_HPP

#include <Eigen/Dense>

namespace sturdr {

class Beamsteer {
 private:
  /**
   * @brief antenna configuration parameters
   */
  int n_ant_;
  double wavelength_;
  Eigen::MatrixXd ant_xyz_body_;

  /**
   * @brief deterministic beam/null stearing parameters
   */
  double null_factor_;
  Eigen::VectorXcd w_beam_;
  Eigen::VectorXcd w_null_;

  /**
   * @brief Least mean squares parameters
   */
  double mu_;
  Eigen::RowVectorXcd W_;

 public:
  /**
   * *=== Beamsteer ===*
   * @brief Constructor
   * @param ant_xyz 3xM antenna relative locations in the body frame
   * @param
   */
  Beamsteer(Eigen::MatrixXd &ant_xyz, bool is_beam = true);

  /**
   * *=== ~Beamsteer ===*
   * @brief Destructor
   */
  ~Beamsteer();

  /**
   * *=== DeterministicBeam ===*
   * @brief Deterministic beamsteering where desired unit vector is provided
   * @param ant_sig signals recorded by each antenna (each column is an antenna)
   * @param u       unit vector in the body frame
   * @return Single combined signal steered in direction of 'u'
   */
  Eigen::VectorXcd DeterministicBeam(
      const Eigen::Ref<const Eigen::MatrixXcd> &ant_sig,
      const Eigen::Ref<const Eigen::Vector3d> &u);

  /**
   * *=== DeterministicNull ===*
   * @brief Deterministic nulling where desired unit vector is provided
   * @param ant_sig signals recorded by each antenna (each column is an antenna)
   * @param u       unit vector in the body frame
   * @return Single combined signal nulled in direction of 'u'
   */
  Eigen::VectorXcd DeterministicNull(
      const Eigen::Ref<const Eigen::MatrixXcd> &ant_sig,
      const Eigen::Ref<const Eigen::Vector3d> &u);

  /**
   * *=== LmsSteer ===*
   * @brief Least-mean-squares beamsteering where optimal unit vector is derived
   * @param X   signals recorded by each antenna (each column is an antenna)
   * @param d   desired signal replica (from tracking loop)
   */
  Eigen::VectorXcd LmsSteer(
      const Eigen::Ref<const Eigen::MatrixXcd> &X, const Eigen::Ref<const Eigen::VectorXcd> &d);

  /**
   * *=== LmsNull ===*
   * @brief Least-mean-squares nulling where optimal unit vector is derived
   *          - this implies the desired replica is 0
   * @param X   signals recorded by each antenna (each column is an antenna)
   */
  Eigen::VectorXcd LmsNull(const Eigen::Ref<const Eigen::MatrixXcd> &X);
};

}  // namespace sturdr

#endif