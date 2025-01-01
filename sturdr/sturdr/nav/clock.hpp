/**
 * *clock.cpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/nav/clock.cpp
 * @brief   Implementation and definitions for calculating clock parameters.
 * @date    December 2024
 * @author  Daniel Sturdivant <sturdivant20@gmail.com>
 * =======  ========================================================================================
 */

#ifndef STURDR_CLOCK_HPP
#define STURDR_CLOCK_HPP

#include <Eigen/Dense>
#include <string>

namespace sturdr {

/**
 * @brief Struct of navigation clock Allan variance values
 */
struct NavigationClock {
  double h0;
  double h1;
  double h2;
};

inline constexpr NavigationClock LOW_QUALITY_TCXO = NavigationClock{2e-19, 7e-21, 2e-20};
inline constexpr NavigationClock HIGH_QUALITY_TCXO = NavigationClock{2e-21, 1e-22, 2e-20};
inline constexpr NavigationClock OCXO = NavigationClock{2e-25, 7e-25, 6e-25};
inline constexpr NavigationClock RUBIDIUM = NavigationClock{2e-22, 4.5e-26, 1e-30};
inline constexpr NavigationClock CESIUM = NavigationClock{2e-22, 5e-27, 1.5e-33};

/**
 * *=== GetNavClock ===*
 * @brief Generator for the default navigation clocks
 * @param clock_name  Name of the desired default clock
 * @return Allan variance parameters for the desired clock
 */
NavigationClock GetNavClock(std::string clock_name);

/**
 * *=== NavClockDynamics ===*
 * @brief Returns the state transition matrix for a navigation clock
 * @param T   Integration period [s]
 * @returns 2x2 matrix to propagate the navigation clock
 */
Eigen::Matrix2d NavClockDynamics(const double &T);

/**
 * *=== NavClockProcessCov ===*
 * @brief Returns the process covariance matrix for the desired navigation clock
 * @param c   Navigation clock structure
 * @param T   Integration period [s]
 * @returns 2x2 covariance matrix due to propagation the navigation clock model
 */
Eigen::Matrix2d NavClockProcessCov(const NavigationClock &c, const double &T);

}  // end namespace sturdr

#endif