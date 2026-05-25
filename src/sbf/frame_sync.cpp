// SPDX-License-Identifier: BSD-3-Clause
//
// SBF frame synchronizer — Phase 3 implementation.
//
// Specification: docs/architecture/03-sbf-parser-design.md §§2, 4, 8.1, 8.2.
//
// --- Design overview --------------------------------------------------------
//
// buf_ is the backing store.  At the start of feed():
//   1. Drain frame_drain_ bytes from the front of buf_ (these are bytes that
//      belonged to frames emitted in the PREVIOUS feed() call; we couldn't
//      erase them during the previous call because Frame::body() spans point
//      directly into buf_ and must stay valid until the caller's next feed()).
//   2. Append the new bytes to the back of buf_.
//   3. Run the state machine with a local `pos` cursor that begins at 0 and
//      advances through buf_ as bytes are consumed.
//
// The state machine:
//
//   Idle           -- advance pos until buf_[pos] == 0x24 ('$').
//   GotDollar      -- buf_[pos] is '$'; need buf_[pos+1] to decide.
//                     '@' -> ReadingHeader (pos += 2, frame_start = pos-2).
//                     '$' -> stay GotDollar, advance one byte (new '$').
//                     other -> drop '$', back to Idle.
//   ReadingHeader  -- need (frame_start+8) bytes total to read CRC/ID/Length.
//   ReadingBody    -- need (frame_start+Length) bytes total.
//                     On length invalid: resync from frame_start+1.
//                     On CRC fail:       resync from frame_start+1.
//                     On CRC match:      emit Frame; frame_drain_ += Length;
//                                        advance pos to frame_start+Length;
//                                        back to Idle.
//
// Buffer lifetime:
//   Frame::body() = ConstByteSpan{&buf_[frame_start+8], Length-8}.
//   These pointers are stable: std::deque guarantees pointer stability for
//   existing elements under push_back and pop_front.
//   We do NOT pop_front within feed(); we defer to the drain at the next
//   feed() call (via frame_drain_).
//   frame_drain_ always equals the TOTAL bytes consumed by all frames emitted
//   during the current feed() call (which happens to equal the largest
//   `pos` value recorded when we emit, since emitted frames are contiguous
//   from the start of the non-drained buf_).
//
// Counter semantics (per §8.4):
//   bytes_consumed   -- incremented by bytes.size() on entry (before processing).
//   frames_emitted   -- incremented on each CRC-valid frame.
//   crc_failed       -- incremented on each CRC mismatch.
//   length_invalid   -- incremented on each invalid Length field.
//   resyncs          -- incremented on each crc_failed OR length_invalid event
//                       (one resync per failure, not per byte skipped).
//
// No spdlog, no std::cout.  Counters only (per §10).
// ---------------------------------------------------------------------------

#include "sbf/frame_sync.hpp"
#include "sbf/crc16_xmodem.hpp"

#include <algorithm>
#include <cassert>
#include <cstring>
#include <vector>

namespace asterx::sbf {

// ---------------------------------------------------------------------------
// Helper: read a LE uint16 from buf_ at offset `off`.
// ---------------------------------------------------------------------------
static inline std::uint16_t
buf_u16le(const std::deque<std::uint8_t>& d, std::size_t off) noexcept {
    return static_cast<std::uint16_t>(
        static_cast<std::uint16_t>(d[off]) |
        (static_cast<std::uint16_t>(d[off + 1]) << 8));
}

// ---------------------------------------------------------------------------
SbfFrameSync::SbfFrameSync()  = default;
SbfFrameSync::~SbfFrameSync() = default;

void SbfFrameSync::reset() noexcept {
    buf_.clear();
    state_       = State::Idle;
    frame_drain_ = 0;
    // stats_ is cumulative; NOT reset here per spec §8.
}

void SbfFrameSync::feed(ConstByteSpan bytes, std::vector<Frame>& out) {
    // -----------------------------------------------------------------------
    // 1. Drain bytes committed to frames in the PREVIOUS feed() call.
    //    This must happen BEFORE we append new bytes so that the pointers in
    //    any Frame objects from the previous call are not invalidated.
    //    (Callers are expected NOT to use those Frame objects after they call
    //    feed() again — but we drain eagerly so buf_ doesn't grow unboundedly.)
    // -----------------------------------------------------------------------
    if (frame_drain_ > 0) {
        assert(frame_drain_ <= buf_.size());
        buf_.erase(buf_.begin(),
                   buf_.begin() + static_cast<std::ptrdiff_t>(frame_drain_));
        frame_drain_ = 0;
    }

    // -----------------------------------------------------------------------
    // 2. Count bytes consumed (cumulative statistic; includes ALL bytes
    //    regardless of whether they yield a valid frame).
    // -----------------------------------------------------------------------
    stats_.bytes_consumed += bytes.size();

    // -----------------------------------------------------------------------
    // 3. Append incoming bytes to buf_.
    // -----------------------------------------------------------------------
    for (std::size_t i = 0; i < bytes.size(); ++i) {
        buf_.push_back(bytes[i]);
    }

    // -----------------------------------------------------------------------
    // 4. Run state machine.
    //
    //    `pos`         -- logical cursor: the first byte not yet committed to
    //                     a decision by the state machine.  It only moves
    //                     forward.
    //    `frame_start` -- index in buf_ where the current '$' was found.
    //                     Meaningful only in GotDollar / ReadingHeader /
    //                     ReadingBody.
    // -----------------------------------------------------------------------
    std::size_t pos         = 0;
    std::size_t frame_start = 0;  // only valid in non-Idle states

    // Restore any partial-frame position.  If state_ != Idle we were already
    // mid-frame when the previous feed() ended; the partial bytes are at the
    // front of buf_ (frame_drain_ was just drained, so buf_[0] is the '$').
    // frame_start is always 0 when re-entering a non-Idle state because buf_
    // is re-anchored after each drain.
    frame_start = 0;

    const std::size_t buf_size = buf_.size();

    bool progress = true;
    while (progress) {
        progress = false;

        switch (state_) {

        // -------------------------------------------------------------------
        case State::Idle: {
            // Advance pos past non-'$' bytes.
            while (pos < buf_size && buf_[pos] != 0x24u) {
                ++pos;
            }
            if (pos < buf_size) {
                // Found '$'.
                frame_start = pos;
                state_      = State::GotDollar;
                progress    = true;
            }
            // else: exhausted input; stall.
            break;
        }

        // -------------------------------------------------------------------
        case State::GotDollar: {
            // buf_[frame_start] == '$'.  Need one more byte.
            if (frame_start + 1 >= buf_size) {
                // Stall; not enough bytes.
                break;
            }
            const std::uint8_t next = buf_[frame_start + 1];
            if (next == 0x40u) {
                // '$@' found — enter ReadingHeader.
                pos      = frame_start + 2;
                state_   = State::ReadingHeader;
                progress = true;
            } else if (next == 0x24u) {
                // Second byte is another '$'; shift frame_start forward by 1.
                // The first '$' was not a valid sync start.
                ++frame_start;
                pos      = frame_start;
                // state_ stays GotDollar
                progress = true;
            } else {
                // Neither '@' nor '$'; the '$' was noise.
                // Advance past it and rescan from Idle.
                pos      = frame_start + 1;
                frame_start = 0;
                state_   = State::Idle;
                progress = true;
            }
            break;
        }

        // -------------------------------------------------------------------
        case State::ReadingHeader: {
            // Need 8 bytes starting at frame_start:
            //   [0..1] sync ('$','@'), [2..3] CRC, [4..5] ID, [6..7] Length.
            if (frame_start + 8 > buf_size) {
                // Stall.
                break;
            }
            // pos is now at frame_start+2 from GotDollar; after the header we
            // will be at frame_start+8.  Update pos now.
            pos = frame_start + 8;

            const std::uint16_t len_field = buf_u16le(buf_, frame_start + 6);

            // Validate Length.
            if ((len_field % 4u != 0u) ||
                (len_field < 8u) ||
                (len_field > kMaxSbfLength)) {
                ++stats_.length_invalid;
                ++stats_.resyncs;
                // Resync: drop the '$' at frame_start, resume scan from +1.
                pos         = frame_start + 1;
                frame_start = 0;
                state_      = State::Idle;
                progress    = true;
                break;
            }

            state_   = State::ReadingBody;
            progress = true;
            break;
        }

        // -------------------------------------------------------------------
        case State::ReadingBody: {
            const std::uint16_t len_field = buf_u16le(buf_, frame_start + 6);
            const std::size_t   total     = static_cast<std::size_t>(len_field);

            if (frame_start + total > buf_size) {
                // Stall; body not complete yet.
                break;
            }

            // Complete frame is in buf_[frame_start .. frame_start+total-1].
            // CRC covers buf_[frame_start+4 .. frame_start+total-1]:
            //   that is, the ID + Length + body (NOT sync, NOT CRC field).
            const std::size_t crc_data_off = frame_start + 4u;
            const std::size_t crc_data_len = total - 4u;

            // Copy to contiguous buffer for CRC (deque is not contiguous).
            // Maximum 8184 bytes.  Stack-allocating 8 KiB risks stack overflow
            // on some platforms; use a small_vector-style approach: stack for
            // small frames, heap for large ones.
            std::uint16_t computed_crc;
            if (crc_data_len <= 256u) {
                std::uint8_t tmp[256];
                for (std::size_t i = 0; i < crc_data_len; ++i) {
                    tmp[i] = buf_[crc_data_off + i];
                }
                computed_crc = crc_ccitt_xmodem(tmp, crc_data_len);
            } else {
                std::vector<std::uint8_t> tmp(crc_data_len);
                for (std::size_t i = 0; i < crc_data_len; ++i) {
                    tmp[i] = buf_[crc_data_off + i];
                }
                computed_crc = crc_ccitt_xmodem(tmp.data(), crc_data_len);
            }

            const std::uint16_t header_crc = buf_u16le(buf_, frame_start + 2u);
            const std::uint16_t id_field   = buf_u16le(buf_, frame_start + 4u);

            if (computed_crc != header_crc) {
                // CRC mismatch: resync from frame_start+1.
                ++stats_.crc_failed;
                ++stats_.resyncs;
                pos         = frame_start + 1;
                frame_start = 0;
                state_      = State::Idle;
                progress    = true;
                break;
            }

            // CRC match: emit Frame.
            //
            // Copy the body bytes into the Frame's owned storage. We must
            // copy element-by-element from the std::deque (the deque is
            // pointer-stable but NOT contiguous: adjacent elements may live
            // in different chunks, so building a span over the deque and
            // dereferencing it past a chunk boundary is UB and produces
            // garbage for any block larger than ~one chunk).
            const std::size_t body_size = total - 8u;
            std::vector<std::uint8_t> body_bytes(body_size);
            for (std::size_t b = 0; b < body_size; ++b) {
                body_bytes[b] = buf_[frame_start + 8u + b];
            }

            out.emplace_back(Frame{
                header_crc,
                id_field,
                len_field,
                std::move(body_bytes)});

            ++stats_.frames_emitted;

            // Schedule drain of frame bytes for the NEXT feed() call.
            // frame_drain_ accumulates across multiple frames emitted in one
            // feed(); it equals the offset just past the last emitted frame
            // relative to the start of buf_ at entry to this feed() (which is
            // 0 after the drain step above).
            frame_drain_ = frame_start + total;  // absolute end of this frame

            // Advance pos and resume scanning for the next frame.
            pos         = frame_start + total;
            frame_start = 0;
            state_      = State::Idle;
            progress    = true;
            break;
        }

        }  // switch
    }  // while

    // -----------------------------------------------------------------------
    // 5. Persist state across feed() calls.
    //
    //    If we ended mid-frame (state_ != Idle), buf_[0] must be the '$' of
    //    the in-progress candidate so that when feed() is called again the
    //    drain step + append + state machine re-entry work correctly.
    //
    //    Specifically: any bytes BEFORE frame_start are fully scanned junk
    //    that we can discard now (they don't belong to a frame candidate).
    //    Bytes [frame_start, buf_size) are the in-progress candidate.
    //    We erase [0, frame_start) so that buf_[0] is the '$' on re-entry.
    //
    //    For Idle state: we can discard everything up to `pos` because those
    //    bytes were either emitted or scanned as junk.  BUT: if any frames
    //    were emitted this call (frame_drain_ > 0), we must not touch the
    //    leading bytes that body_ spans point into.  In Idle state with
    //    frame_drain_ > 0, pos >= frame_drain_ (we already advanced past all
    //    emitted frames), so we must not erase beyond frame_drain_ here.
    //    Leave the drain to the NEXT feed() call (frame_drain_ already set).
    //
    //    Summary:
    //      - In non-Idle: erase [0, frame_start) unconditionally (junk before
    //        the current candidate).  Note: frame_drain_ CAN be > 0 here —
    //        we may have emitted one or more frames in this feed() and then
    //        re-entered a non-Idle state on a trailing partial frame.  In
    //        that case frame_start (= position of the trailing '$') is
    //        always >= frame_drain_ (= end of the last emitted frame),
    //        because Idle scanning never moves pos backwards.  So erasing
    //        [0, frame_start) supersedes the deferred drain — we must reset
    //        frame_drain_ to 0, otherwise the NEXT feed() will erase that
    //        many bytes again, walking past _M_finish and silently
    //        corrupting buf_'s internal node map (surfaces as a glibc
    //        "double free detected in tcache" hours later).  Fixed
    //        2026-05-25 after long-soak crash on real receiver data.
    //      - In Idle with frame_drain_ == 0: erase [0, pos) (all scanned junk).
    //      - In Idle with frame_drain_ > 0: leave buf_ intact; drain deferred.
    // -----------------------------------------------------------------------
    if (state_ != State::Idle) {
        // Mid-frame: trim junk prefix before frame_start.
        if (frame_start > 0) {
            assert(frame_drain_ <= frame_start);
            buf_.erase(buf_.begin(),
                       buf_.begin() + static_cast<std::ptrdiff_t>(frame_start));
            // The just-erased prefix [0, frame_start) covers the entire
            // deferred-drain region [0, frame_drain_), so the deferred drain
            // is now satisfied; clear it to prevent a double-erase on entry
            // to the next feed().
            frame_drain_ = 0;
        }
    } else {
        if (frame_drain_ == 0) {
            // No emitted frames: discard everything we scanned as junk.
            if (pos > 0 && pos <= buf_.size()) {
                buf_.erase(buf_.begin(),
                           buf_.begin() + static_cast<std::ptrdiff_t>(pos));
            }
        }
        // else: frame_drain_ > 0 — leave buf_ intact; drain at next feed().
    }
}

}  // namespace asterx::sbf
