/**
 * *data-type-adapters.hpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/data-type-adapters.hpp
 * @brief   STURDR data-type-adapters.
 * @date    December 2024
 * =======  ========================================================================================
 */

#pragma once

#ifndef STURDR_DATA_TYPE_ADAPTERS_HPP
#define STURDR_DATA_TYPE_ADAPTERS_HPP

#include <complex>
#include <cstdint>
#include <functional>

namespace sturdr {

// /**
//  * *=== DataToDouble ===*
//  * @brief convert array of T to doubles
//  */
// template <int Size, typename T>
// void DataToDouble(const T in[], double out[]) {
//   for (int i = 0; i < Size; i++) {
//     out[i] = static_cast<double>(in[i]);
//   }
// };

// /**
//  * *=== DataToIDouble ===*
//  * @brief convert array of T to doubles
//  */
// template <int Size, typename T>
// void DataToIDouble(const T in[], std::complex<double> out[]) {
//   for (int i = 0; i < Size; i++) {
//     out[i] = static_cast<double>(in[i]);
//   }
// };

// /**
//  * *=== IDataToIDouble ===*
//  * @brief convert array of T to doubles
//  */
// template <int Size, typename T>
// void IDataToIDouble(const std::complex<T> in[], std::complex<double> out[]) {
//   for (int i = 0; i < Size; i++) {
//     out[i] =
//         std::complex<double>(static_cast<double>(in[i].real()),
//         static_cast<double>(in[i].imag()));
//   }
// };

union DataTypeAdapter {
  std::function<void(int8_t[], double[])> ByteToDouble;
  std::function<void(int16_t[], double[])> ShortToDouble;
  std::function<void(int8_t[], std::complex<double>[])> ByteToIDouble;
  std::function<void(int16_t[], std::complex<double>[])> ShortToIDouble;
  std::function<void(std::complex<int8_t>[], std::complex<double>[])> IByteToIDouble;
  std::function<void(std::complex<int16_t>[], std::complex<double>[])> IShortToIDouble;
};

/**
 * *=== ByteToDouble ===*
 * @brief convert array of bytes (int8) to doubles
 */
template <int Size>
void ByteToDouble(const int8_t in[], double out[]) {
  for (int i = 0; i < Size; i++) {
    out[i] = static_cast<double>(in[i]);
  }
};

/**
 * *=== ShortToDouble ===*
 * @brief convert array of shorts (int16) to doubles
 */
template <int Size>
void ShortToDouble(const int16_t in[], double out[]) {
  for (int i = 0; i < Size; i++) {
    out[i] = static_cast<double>(in[i]);
  }
};

/**
 * *=== ByteToIDouble ===*
 * @brief convert array of bytes (int8) to complex doubles
 */
template <int Size>
void ByteToIDouble(const int8_t in[], std::complex<double> out[]) {
  for (int i = 0; i < Size; i++) {
    out[i] = static_cast<std::complex<double>>(in[i]);
  }
};

/**
 * *=== ShortToIDouble ===*
 * @brief convert array of shorts (int16) to complex doubles
 */
template <int Size>
void ShortToIDouble(const int16_t in[], std::complex<double> out[]) {
  for (int i = 0; i < Size; i++) {
    out[i] = static_cast<std::complex<double>>(in[i]);
  }
};

/**
 * *=== IByteToIDouble ===*
 * @brief convert array of complex bytes (int8) to complex doubles
 */
template <int Size>
void IByteToIDouble(const std::complex<int8_t> in[], std::complex<double> out[]) {
  for (int i = 0; i < Size; i++) {
    out[i] =
        std::complex<double>(static_cast<double>(in[i].real()), static_cast<double>(in[i].imag()));
  }
};

/**
 * *=== IShortToIDouble ===*
 * @brief convert array of complex shorts (int16) to complex doubles
 */
template <int Size>
void IShortToIDouble(const std::complex<int16_t> in[], std::complex<double> out[]) {
  for (int i = 0; i < Size; i++) {
    out[i] =
        std::complex<double>(static_cast<double>(in[i].real()), static_cast<double>(in[i].imag()));
  }
};

}  // namespace sturdr

#endif