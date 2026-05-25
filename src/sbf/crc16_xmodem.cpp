// SPDX-License-Identifier: BSD-3-Clause
//
// Out-of-line anchor for crc16_xmodem.hpp.  The header is constexpr-only,
// but having a .cpp ensures the symbol participates in the build target's
// translation-unit list and keeps source listings ordered.
//
#include "sbf/crc16_xmodem.hpp"

namespace asterx::sbf {

// The crc_ccitt_xmodem function is fully constexpr in the header; no
// runtime support is needed here.  This file is intentionally empty other
// than the include.

}  // namespace asterx::sbf
