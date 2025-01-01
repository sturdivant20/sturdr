#include <array>
#include <cassert>
#include <cstdint>
#include <navtools/binary-ops.hpp>

namespace sturdr {
void CodeGenCA(std::array<bool, 1023>& sequence, const uint8_t prn) {
  assert(!((prn < 1) || (prn > 32)));

  static constexpr uint8_t g2_out_taps[32][2] = {
      /*01 {2, 6} */ {1, 5},
      /*02 {3, 7} */ {2, 6},
      /*03 {4, 8} */ {3, 7},
      /*04 {5, 9} */ {4, 8},
      /*05 {1, 9} */ {0, 8},
      /*06 {2, 10}*/ {1, 9},
      /*07 {1, 8} */ {0, 7},
      /*08 {2, 9} */ {1, 8},
      /*09 {3, 10}*/ {2, 9},
      /*10 {2, 3} */ {1, 2},
      /*11 {3, 4} */ {2, 3},
      /*12 {5, 6} */ {4, 5},
      /*13 {6, 7} */ {5, 6},
      /*14 {7, 8} */ {6, 7},
      /*15 {8, 9} */ {7, 8},
      /*16 {9, 10}*/ {8, 9},
      /*17 {1, 4} */ {0, 3},
      /*18 {2, 5} */ {1, 4},
      /*19 {3, 6} */ {2, 5},
      /*20 {4, 7} */ {3, 6},
      /*21 {5, 8} */ {4, 7},
      /*22 {6, 9} */ {5, 8},
      /*23 {1, 3} */ {0, 2},
      /*24 {4, 6} */ {3, 5},
      /*25 {5, 7} */ {4, 6},
      /*26 {6, 8} */ {5, 7},
      /*27 {7, 9} */ {6, 8},
      /*28 {8, 10}*/ {7, 9},
      /*29 {1, 6} */ {0, 5},
      /*30 {2, 7} */ {1, 6},
      /*31 {3, 8} */ {2, 7},
      /*32 {4, 9} */ {3, 8}};

  // Linear-feedback shift registers
  uint32_t G1 = 0xFFFF;
  uint32_t G2 = 0xFFFF;

  uint8_t taps1[2] = {2, 9};              // 3,10
  uint8_t taps2[6] = {1, 2, 5, 7, 8, 9};  // 2,3,6,8,9,10

  for (std::size_t i = 0; i < 1023; i++) {
    // set value in sequence
    sequence[i] = navtools::CheckBit<true>(G1, 9u) ^
                  navtools::CheckBit<true>(G2, g2_out_taps[prn - 1][0]) ^
                  navtools::CheckBit<true>(G2, g2_out_taps[prn - 1][1]);

    // shift the registers and set first bits
    bool feedback1 = navtools::MultiXor<2, true>(G1, taps1);
    bool feedback2 = navtools::MultiXor<6, true>(G2, taps2);
    G1 <<= 1;
    G2 <<= 1;
    navtools::ModifyBit<true>(G1, 0, feedback1);
    navtools::ModifyBit<true>(G2, 0, feedback2);
  }
};

}  // end namespace sturdr