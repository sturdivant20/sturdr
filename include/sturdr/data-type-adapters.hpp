/**
 * *data-type-adapters.hpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/data-type-adapters.hpp
 * @brief   STURDR data-type-adapters.
 * @date    January 2025
 * =======  ========================================================================================
 */

#ifndef STURDR_DATA_TYPE_ADAPTERS_HPP
#define STURDR_DATA_TYPE_ADAPTERS_HPP

#include <complex>
#include <cstdint>

namespace sturdr {

// union DataTypeAdapter {
//   void (*ByteToDouble)(int8_t[], double[]);
//   void (*ShortToDouble)(int16_t[], double[]);
//   void (*ByteToIDouble)(int8_t[], std::complex<double>[]);
//   void (*ShortToIDouble)(int16_t[], std::complex<double>[]);
//   void (*IByteToIDouble)(std::complex<int8_t>[], std::complex<double>[]);
//   void (*IShortToIDouble)(std::complex<int8_t>[], std::complex<double>[]);
// };

// *=== TypeToDouble ===*
template <typename T>
void TypeToIDouble(const T in[], std::complex<double> out[], const int &len) {
  for (int i = 0; i < len; i++) {
    out[i] = static_cast<std::complex<double>>(in[i]);
  }
}

// *=== ITypeToDouble ===*
template <typename T>
void ITypeToIDouble(const std::complex<T> in[], std::complex<double> out[], const int &len) {
  for (int i = 0; i < len; i++) {
    out[i] = static_cast<std::complex<double>>(in[i].real(), in[i].imag());
  }
}

/**
 * *=== ByteToDouble ===*
 * @brief convert array of bytes (int8) to doubles
 */
void ByteToDouble(const int8_t in[], double out[], const int &len);

/**
 * *=== ShortToDouble ===*
 * @brief convert array of shorts (int16) to doubles
 */
void ShortToDouble(const int16_t in[], double out[], const int &len);

/**
 * *=== ByteToIDouble ===*
 * @brief convert array of bytes (int8) to complex doubles
 */
void ByteToIDouble(const int8_t in[], std::complex<double> out[], const int &len);

/**
 * *=== ShortToIDouble ===*
 * @brief convert array of shorts (int16) to complex doubles
 */
void ShortToIDouble(const int16_t in[], std::complex<double> out[], const int &len);

/**
 * *=== IByteToIDouble ===*
 * @brief convert array of complex bytes (int8) to complex doubles
 */
void IByteToIDouble(const std::complex<int8_t> in[], std::complex<double> out[], const int &len);

/**
 * *=== IShortToIDouble ===*
 * @brief convert array of complex shorts (int16) to complex doubles
 */
void IShortToIDouble(const std::complex<int16_t> in[], std::complex<double> out[], const int &len);

}  // namespace sturdr

#endif