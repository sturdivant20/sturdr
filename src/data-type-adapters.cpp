/**
 * *data-type-adapters.cpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/data-type-adapters.cpp
 * @brief   STURDR data-type-adapters.
 * @date    January 2025
 * =======  ========================================================================================
 */

#include "sturdr/data-type-adapters.hpp"

namespace sturdr {

// *=== ByteToDouble ===*
void ByteToDouble(const int8_t in[], double out[], const int &len) {
  for (int i = 0; i < len; i++) {
    out[i] = static_cast<double>(in[i]);
  }
}

// *=== ShortToDouble ===*
void ShortToDouble(const int16_t in[], double out[], const int &len) {
  for (int i = 0; i < len; i++) {
    out[i] = static_cast<double>(in[i]);
  }
}

// *=== ByteToIDouble ===*
void ByteToIDouble(const int8_t in[], std::complex<double> out[], const int &len) {
  for (int i = 0; i < len; i++) {
    out[i] = static_cast<std::complex<double>>(in[i]);
  }
}

// *=== ShortToIDouble ===*
void ShortToIDouble(const int16_t in[], std::complex<double> out[], const int &len) {
  for (int i = 0; i < len; i++) {
    out[i] = static_cast<std::complex<double>>(in[i]);
  }
}

// *=== IByteToIDouble ===*
void IByteToIDouble(const std::complex<int8_t> in[], std::complex<double> out[], const int &len) {
  for (int i = 0; i < len; i++) {
    out[i] =
        std::complex<double>(static_cast<double>(in[i].real()), static_cast<double>(in[i].imag()));
  }
}

// *=== IShortToIDouble ===*
void IShortToIDouble(const std::complex<int16_t> in[], std::complex<double> out[], const int &len) {
  for (int i = 0; i < len; i++) {
    out[i] =
        std::complex<double>(static_cast<double>(in[i].real()), static_cast<double>(in[i].imag()));
  }
}

}  // namespace sturdr