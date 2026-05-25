// SPDX-License-Identifier: BSD-3-Clause
//
// Unit tests for the CRC-CCITT-XMODEM implementation.
//
// Test vectors:
//   "123456789" -> 0x31C3   (canonical XMODEM check value)
//   ""          -> 0x0000   (initial value)
//   "A"         -> hand-computed
//
// Reference: https://reveng.sourceforge.io/crc-catalogue/16.htm
// (the "CRC-16/XMODEM" entry).
//
#include <array>
#include <cstdint>
#include <cstring>

#include <gtest/gtest.h>

#include "sbf/crc16_xmodem.hpp"

namespace {

std::uint16_t crc_of(const char* s) {
    return asterx::sbf::crc_ccitt_xmodem(
        reinterpret_cast<const std::uint8_t*>(s),
        std::strlen(s));
}

TEST(Crc16Xmodem, EmptyInputYieldsZero) {
    EXPECT_EQ(crc_of(""), 0x0000u);
}

TEST(Crc16Xmodem, CanonicalCheckString) {
    // Per the reveng catalogue, CRC-16/XMODEM("123456789") == 0x31C3.
    EXPECT_EQ(crc_of("123456789"), 0x31C3u);
}

TEST(Crc16Xmodem, SingleByteA) {
    // Hand-computed: CRC for 'A' (0x41) starting from 0x0000.
    // crc ^= 0x41 << 8 = 0x4100.
    // Bit 15 = 0 -> shift = 0x8200.
    // Bit 15 = 1 -> shift+xor poly:
    //   0x8200 << 1 = 0x10400 -> 0x0400 ^ 0x1021 = 0x1421.
    // Bit 15 of 0x1421 = 0 -> 0x2842.
    // Bit 15 = 0 -> 0x5084.
    // Bit 15 = 0 -> 0xA108.
    // Bit 15 = 1 -> (0xA108 << 1) & 0xFFFF = 0x4210; ^ 0x1021 = 0x5231.
    // Bit 15 = 0 -> 0xA462.
    // Bit 15 = 1 -> (0xA462 << 1) & 0xFFFF = 0x48C4; ^ 0x1021 = 0x58E5.
    // Final CRC = 0x58E5.
    EXPECT_EQ(crc_of("A"), 0x58E5u);
}

TEST(Crc16Xmodem, AllZerosBufferOfLengthN) {
    // CRC of any block of all-zero bytes starting from init=0x0000 is 0.
    std::array<std::uint8_t, 32> zeros{};
    EXPECT_EQ(asterx::sbf::crc_ccitt_xmodem(zeros.data(), zeros.size()), 0u);
}

TEST(Crc16Xmodem, KnownStringT) {
    // CRC-16/XMODEM("T"), hand-computed against poly=0x1021, init=0x0000.
    // Consistent with the canonical "123456789" -> 0x31C3 check (above).
    EXPECT_EQ(crc_of("T"), 0x1A71u);
}

TEST(Crc16Xmodem, KnownStringTest) {
    // CRC-16/XMODEM("Test") = 0xAC48 (T=0x1A71, then mix e/s/t).
    EXPECT_EQ(crc_of("Test"), 0xAC48u);
}

}  // namespace
