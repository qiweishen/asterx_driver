// SPDX-License-Identifier: BSD-3-Clause
//
// CRC-CCITT-XMODEM (CRC-16/XMODEM).
//
// Polynomial:  0x1021
// Initial:     0x0000
// Reflect in:  no
// Reflect out: no
// XOR out:     0x0000
//
// Per docs/architecture/03-sbf-parser-design.md §2.  This is the SBF
// frame CRC; do NOT confuse with CRC-CCITT-FALSE (initial 0xFFFF).
//
// Implementation is constexpr-friendly so the 256-byte lookup table is
// computed at compile time.  No third-party dependency.
//
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace asterx::sbf {

namespace detail {

inline constexpr std::array<std::uint16_t, 256> make_crc_table() noexcept {
    std::array<std::uint16_t, 256> table{};
    for (std::size_t i = 0; i < 256; ++i) {
        std::uint16_t c = static_cast<std::uint16_t>(i << 8);
        for (int j = 0; j < 8; ++j) {
            c = static_cast<std::uint16_t>(
                (c & 0x8000) ? static_cast<std::uint16_t>((c << 1) ^ 0x1021)
                             : static_cast<std::uint16_t>(c << 1));
        }
        table[i] = c;
    }
    return table;
}

inline constexpr auto kCrcTable = make_crc_table();

}  // namespace detail

/// Compute the CRC-CCITT-XMODEM of `len` bytes starting at `data`.
///
/// Per SBF, the CRC covers bytes [4 .. 4+Length-1) of the frame --- i.e.
/// the ID and Length fields plus the body.  It does NOT cover the sync
/// bytes (offset 0..1) nor the CRC field itself (offset 2..3).  This
/// function is agnostic to the byte ranges; callers are responsible for
/// supplying the right slice.
[[nodiscard]] constexpr std::uint16_t crc_ccitt_xmodem(
    const std::uint8_t* data, std::size_t len) noexcept {
    std::uint16_t crc = 0x0000;
    for (std::size_t i = 0; i < len; ++i) {
        crc = static_cast<std::uint16_t>(
            (crc << 8) ^ detail::kCrcTable[(crc >> 8) ^ data[i]]);
    }
    return crc;
}

}  // namespace asterx::sbf
