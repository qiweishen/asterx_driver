// SPDX-License-Identifier: BSD-3-Clause
//
// SBF frame synchronizer.  Per docs/architecture/03-sbf-parser-design.md §4.
//
// Phase 2B ships the public interface only; Phase 3 implements the state
// machine (idle / got_dollar / reading_header / reading_body) and the
// resync logic.
//
// Phase 3 amendment: added private `frame_drain_` counter to track how many
// bytes at the front of buf_ belong to already-emitted (but not yet erased)
// frames.  This is required to honour the contract "Frame::body() is valid
// until the next feed() call" while keeping buf_ as the sole backing store
// (i.e. not copying body bytes into Frame).  frame_drain_ is private
// implementation detail; the public surface is unchanged.
//
#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

#include "asterx/byte_view.hpp"

namespace asterx::sbf {

/// Project-wide non-owning byte view (C++17 stand-in for std::span).
using ConstByteSpan = asterx::ConstByteSpan;

/// Maximum SBF block length (header + body).  Per manual, no block
/// exceeds 8188 bytes; we cap parsing to refuse maliciously large frames.
inline constexpr std::uint16_t kMaxSbfLength = 8188;

struct FrameSyncStats {
    std::uint64_t bytes_consumed{0};
    std::uint64_t frames_emitted{0};
    std::uint64_t crc_failed{0};
    std::uint64_t length_invalid{0};
    std::uint64_t resyncs{0};
};

/// A successfully-validated SBF frame.  The body span points into the
/// SbfFrameSync's internal buffer; valid only until the next feed() call.
class Frame {
public:
    [[nodiscard]] std::uint16_t crc()      const noexcept { return crc_; }
    [[nodiscard]] std::uint16_t rawId()    const noexcept { return raw_id_; }
    [[nodiscard]] std::uint16_t blockId()  const noexcept { return raw_id_ & 0x1FFFu; }
    [[nodiscard]] std::uint8_t  blockRev() const noexcept {
        return static_cast<std::uint8_t>(raw_id_ >> 13);
    }
    [[nodiscard]] std::uint16_t length()   const noexcept { return length_; }
    [[nodiscard]] ConstByteSpan body()     const noexcept {
        return ConstByteSpan{body_storage_.data(), body_storage_.size()};
    }
    [[nodiscard]] const std::uint8_t* bodyEnd() const noexcept {
        return body_storage_.data() + body_storage_.size();
    }

private:
    // Frame owns its body bytes in a contiguous std::vector. Earlier versions
    // held a ConstByteSpan into SbfFrameSync's std::deque buffer; that was
    // undefined behaviour: deque elements are pointer-stable but adjacent
    // elements are not guaranteed to be contiguous in memory, so any read
    // past a chunk boundary returned garbage. See the recorder bug surfaced
    // by python-side CRC re-validation on 2026-05-22.
    Frame(std::uint16_t crc,
          std::uint16_t raw_id,
          std::uint16_t length,
          std::vector<std::uint8_t> body)
        : crc_(crc), raw_id_(raw_id), length_(length),
          body_storage_(std::move(body)) {}

    std::uint16_t crc_;
    std::uint16_t raw_id_;
    std::uint16_t length_;
    std::vector<std::uint8_t> body_storage_;

    friend class SbfFrameSync;
};

class SbfFrameSync {
public:
    SbfFrameSync();
    ~SbfFrameSync();

    SbfFrameSync(const SbfFrameSync&)            = delete;
    SbfFrameSync& operator=(const SbfFrameSync&) = delete;

    /// Feed an arbitrary byte chunk; emits zero or more frames into `out`.
    /// The Frame body views are valid until the next call to feed() or
    /// destruction.
    void feed(ConstByteSpan bytes, std::vector<Frame>& out);

    [[nodiscard]] const FrameSyncStats& stats() const noexcept { return stats_; }

    /// Discard any in-progress frame.  Called on transport reconnect.
    void reset() noexcept;

private:
    enum class State : std::uint8_t {
        Idle,
        GotDollar,
        ReadingHeader,
        ReadingBody,
    };

    std::deque<std::uint8_t> buf_;
    State                    state_{State::Idle};
    FrameSyncStats           stats_{};

    // Number of bytes at the front of buf_ that belong to already-emitted
    // frames.  These are drained at the start of the next feed() call so
    // that Frame::body() spans remain valid for the caller's use after the
    // previous feed() returns but before the next one is invoked.
    std::size_t              frame_drain_{0};
};

}  // namespace asterx::sbf
