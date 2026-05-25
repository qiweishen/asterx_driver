// SPDX-License-Identifier: BSD-3-Clause
//
// Unit tests for SbfFrameSync.
//
// All test frames are synthetic (hand-crafted).  CRCs are computed using the
// same crc_ccitt_xmodem function under test, which is cross-verified against
// the canonical check string "123456789" -> 0x31C3 in test_crc16_xmodem.cpp.
// The CRC correctness tests there establish the algorithm is correct; here we
// rely on it to produce self-consistent frames.
//
// Test-frame generation method:
//   1. Choose block_id (e.g. 0x09D2 = 2514, a synthetic value within 13 bits).
//   2. Choose a deterministic body (e.g. 0x00, 0x01, ..., 0x0F for 16 bytes).
//   3. Length = 8 (header) + body_size.  Must be multiple of 4.
//   4. Write the 8-byte header into a buffer:
//        [0,1] = 0x24, 0x40  (sync)
//        [2,3] = CRC (computed last, placeholder 0,0 initially)
//        [4,5] = block_id LE
//        [6,7] = Length LE
//   5. Append body bytes.
//   6. Compute CRC over bytes [4..Length-1] with crc_ccitt_xmodem.
//   7. Write CRC into bytes [2..3] LE.
//
// This method is implemented in the helper build_frame() below.
//
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

#include <gtest/gtest.h>

// Access the private src/ tree (permitted by CMakeLists via include_directories).
#include "sbf/frame_sync.hpp"
#include "sbf/crc16_xmodem.hpp"

namespace {

// ---------------------------------------------------------------------------
// Helper: build a valid SBF frame with the given block_id and body bytes.
// Returns the complete frame bytes (header + body) with correct CRC.
// ---------------------------------------------------------------------------
static std::vector<std::uint8_t>
build_frame(std::uint16_t block_id,
            const std::vector<std::uint8_t>& body)
{
    // body_size must be a multiple of 4, >= 0.
    // pad body to next multiple of 4 if needed.
    std::vector<std::uint8_t> padded_body = body;
    while (padded_body.size() % 4 != 0) {
        padded_body.push_back(0x00);
    }

    const std::uint16_t total =
        static_cast<std::uint16_t>(8u + padded_body.size());

    std::vector<std::uint8_t> frame(total);
    // sync
    frame[0] = 0x24u;
    frame[1] = 0x40u;
    // CRC placeholder
    frame[2] = 0x00u;
    frame[3] = 0x00u;
    // ID (block_id in low 13 bits; no revision bits set)
    frame[4] = static_cast<std::uint8_t>(block_id & 0xFFu);
    frame[5] = static_cast<std::uint8_t>((block_id >> 8) & 0xFFu);
    // Length
    frame[6] = static_cast<std::uint8_t>(total & 0xFFu);
    frame[7] = static_cast<std::uint8_t>((total >> 8) & 0xFFu);
    // Body
    std::copy(padded_body.begin(), padded_body.end(), frame.begin() + 8);

    // Compute CRC over bytes [4..total-1].
    const std::uint16_t crc =
        asterx::sbf::crc_ccitt_xmodem(frame.data() + 4u,
                                       static_cast<std::size_t>(total) - 4u);
    frame[2] = static_cast<std::uint8_t>(crc & 0xFFu);
    frame[3] = static_cast<std::uint8_t>((crc >> 8) & 0xFFu);

    return frame;
}

// Convenience overload: empty body.
static std::vector<std::uint8_t> build_frame(std::uint16_t block_id) {
    return build_frame(block_id, {});
}

// Helper: feed all bytes in one call.
static void feed_all(asterx::sbf::SbfFrameSync& sync,
                     const std::vector<std::uint8_t>& data,
                     std::vector<asterx::sbf::Frame>& out)
{
    sync.feed(asterx::ConstByteSpan{data.data(), data.size()}, out);
}

// ---------------------------------------------------------------------------
// Test suite: FrameSync
// ---------------------------------------------------------------------------

// TC1: Happy path — minimal valid frame, empty body (Length == 8).
//
// A frame with Length=8 has no body bytes.  This is the smallest legal SBF
// frame (multiples-of-4 rule: 8 is a multiple of 4; >= 8; <= 8188).
// We expect exactly one Frame emitted with body.size() == 0.
TEST(FrameSync, HappyPathEmptyBody) {
    asterx::sbf::SbfFrameSync sync;
    std::vector<asterx::sbf::Frame> out;

    const auto frame_bytes = build_frame(0x1234u);
    ASSERT_EQ(frame_bytes.size(), 8u);  // Length==8, body==0

    feed_all(sync, frame_bytes, out);

    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].blockId(), 0x1234u & 0x1FFFu);
    EXPECT_EQ(out[0].length(), 8u);
    EXPECT_EQ(out[0].body().size(), 0u);
    EXPECT_EQ(sync.stats().frames_emitted, 1u);
    EXPECT_EQ(sync.stats().crc_failed,     0u);
    EXPECT_EQ(sync.stats().length_invalid, 0u);
    EXPECT_EQ(sync.stats().resyncs,        0u);
}

// TC2: Happy path — frame with a 16-byte body (ExtSensorMeas-shaped).
//
// block_id = 4050 (ExtSensorMeas), body = 16 deterministic bytes.
// Length = 8 + 16 = 24 (multiple of 4).  Assert body contents are intact.
TEST(FrameSync, HappyPathExtSensorMeasSized) {
    asterx::sbf::SbfFrameSync sync;
    std::vector<asterx::sbf::Frame> out;

    const std::uint16_t block_id = 4050u;
    const std::vector<std::uint8_t> body = {
        0x00, 0x01, 0x02, 0x03,
        0x10, 0x11, 0x12, 0x13,
        0xAA, 0xBB, 0xCC, 0xDD,
        0xDE, 0xAD, 0xBE, 0xEF
    };
    const auto frame_bytes = build_frame(block_id, body);

    ASSERT_EQ(frame_bytes.size(), 24u);

    feed_all(sync, frame_bytes, out);

    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].blockId(),      block_id);
    EXPECT_EQ(out[0].length(),       24u);
    EXPECT_EQ(out[0].body().size(),  16u);

    // Verify body content byte-by-byte.
    for (std::size_t i = 0; i < body.size(); ++i) {
        EXPECT_EQ(out[0].body()[i], body[i])
            << "body byte mismatch at index " << i;
    }

    EXPECT_EQ(sync.stats().frames_emitted, 1u);
    EXPECT_EQ(sync.stats().crc_failed,     0u);
    EXPECT_EQ(sync.stats().length_invalid, 0u);
    EXPECT_EQ(sync.stats().resyncs,        0u);
    EXPECT_EQ(sync.stats().bytes_consumed, static_cast<std::uint64_t>(frame_bytes.size()));
}

// TC3: Garbage-prefix recovery.
//
// Feed N bytes of 0xAB (none of which is 0x24) followed by a valid frame.
// The parser must skip the garbage, find the frame, and emit exactly one
// Frame.  No resyncs are expected because we never "claimed" a sync on the
// garbage bytes (they weren't '$').
TEST(FrameSync, GarbagePrefixRecovery) {
    asterx::sbf::SbfFrameSync sync;
    std::vector<asterx::sbf::Frame> out;

    const std::size_t garbage_len = 42u;
    const std::vector<std::uint8_t> body = {0x01, 0x02, 0x03, 0x04};
    const auto frame_bytes = build_frame(0x0010u, body);

    std::vector<std::uint8_t> input(garbage_len, 0xABu);
    input.insert(input.end(), frame_bytes.begin(), frame_bytes.end());

    feed_all(sync, input, out);

    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].blockId(), 0x0010u);
    EXPECT_GE(sync.stats().bytes_consumed,
              static_cast<std::uint64_t>(garbage_len + frame_bytes.size()));
    EXPECT_EQ(sync.stats().frames_emitted, 1u);
    // No sync was ever "claimed" on the 0xAB bytes, so resyncs should be 0.
    EXPECT_EQ(sync.stats().resyncs, 0u);
}

// TC4: Truncated-frame recovery.
//
// Feed the first half of a valid frame (including the header so the state
// machine enters ReadingBody), then in a second feed() call provide a
// completely different valid frame.
//
// Expected outcome:
//   * The first half-frame never emits (it never completes).
//   * When the second frame's bytes arrive, the state machine has more bytes
//     than the first frame declared; CRC fails; it resyncs and finds the
//     second frame's '$@'.  The second frame is emitted.
//   * crc_failed == 1, resyncs >= 1.
//
// Note: this behaviour depends on the first frame's Length fitting within the
// combined buffer that arrives on the second feed().  We choose the first
// frame's body to be small (8 bytes) so the second frame's bytes complete it.
TEST(FrameSync, TruncatedFrameRecovery) {
    asterx::sbf::SbfFrameSync sync;
    std::vector<asterx::sbf::Frame> out;

    // Frame A: 24 bytes total (8 header + 16 body).
    const std::vector<std::uint8_t> body_a(16u, 0xAAu);
    const auto frame_a = build_frame(0x0001u, body_a);
    ASSERT_EQ(frame_a.size(), 24u);

    // Frame B: different block_id, 12 bytes total (8 header + 4 body).
    const std::vector<std::uint8_t> body_b = {0x11, 0x22, 0x33, 0x44};
    const auto frame_b = build_frame(0x0002u, body_b);
    ASSERT_EQ(frame_b.size(), 12u);

    // Feed only the first 12 bytes of frame A (header + 4 body bytes).
    const std::size_t half = 12u;
    std::vector<std::uint8_t> first_half(frame_a.begin(), frame_a.begin() + half);

    feed_all(sync, first_half, out);

    // No frame should be emitted yet (body incomplete).
    EXPECT_EQ(out.size(), 0u);
    EXPECT_EQ(sync.stats().frames_emitted, 0u);

    // Now feed frame B in its entirety.
    feed_all(sync, frame_b, out);

    // Expected: 0 emitted from frame A (CRC mismatch after body is "completed"
    // by frame B's bytes, then resync finds frame B's '$@').
    // frame B should be emitted.
    EXPECT_EQ(sync.stats().frames_emitted, 1u);
    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].blockId(), 0x0002u);

    // At least one CRC failure (from the corrupted "first frame" whose body
    // was filled by frame B's bytes) and at least one resync.
    EXPECT_GE(sync.stats().crc_failed, 1u);
    EXPECT_GE(sync.stats().resyncs,    1u);
}

// TC5: Bad-CRC drop — flip one body byte, verify failure, then verify
// recovery on the next valid frame.
TEST(FrameSync, BadCrcDropAndRecovery) {
    asterx::sbf::SbfFrameSync sync;
    std::vector<asterx::sbf::Frame> out;

    const std::vector<std::uint8_t> body = {0xDE, 0xAD, 0xBE, 0xEF};
    auto bad_frame = build_frame(0x00FFu, body);

    // Flip one body byte (byte index 8 is the first body byte).
    bad_frame[8] ^= 0xFFu;

    feed_all(sync, bad_frame, out);

    EXPECT_EQ(out.size(),             0u);
    EXPECT_EQ(sync.stats().crc_failed, 1u);
    EXPECT_GE(sync.stats().resyncs,    1u);
    EXPECT_EQ(sync.stats().frames_emitted, 0u);

    // Feed a valid frame immediately after; parser must NOT be stuck.
    const auto good_frame = build_frame(0x00FFu, body);
    feed_all(sync, good_frame, out);

    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].blockId(), 0x00FFu);
    EXPECT_EQ(sync.stats().frames_emitted, 1u);
    EXPECT_EQ(sync.stats().crc_failed,     1u);  // still 1; no new failure
}

// TC6: Length-invalid drop — feed a frame whose Length field is not a
// multiple of 4.  Parser must increment length_invalid, not hang, and
// recover on the next valid frame.
TEST(FrameSync, LengthInvalidDrop) {
    asterx::sbf::SbfFrameSync sync;
    std::vector<asterx::sbf::Frame> out;

    // Craft a frame with Length = 11 (not a multiple of 4).
    // We can't use build_frame() because it enforces alignment.
    // Build manually.
    std::vector<std::uint8_t> bad_frame = {
        0x24u, 0x40u,       // sync
        0x00u, 0x00u,       // CRC placeholder (irrelevant; length check comes first)
        0x42u, 0x00u,       // ID = 0x0042
        0x0Bu, 0x00u,       // Length = 11 (NOT a multiple of 4)
        0x01u, 0x02u, 0x03u // 3 body bytes (total 11)
    };
    // CRC is irrelevant; the length check fires before CRC computation.
    // (Leave CRC as 0x0000.)

    feed_all(sync, bad_frame, out);

    EXPECT_EQ(out.size(),                   0u);
    EXPECT_EQ(sync.stats().length_invalid,  1u);
    EXPECT_GE(sync.stats().resyncs,         1u);
    EXPECT_EQ(sync.stats().frames_emitted,  0u);
    EXPECT_EQ(sync.stats().crc_failed,      0u);

    // Feed a valid frame; parser must recover.
    const std::vector<std::uint8_t> body = {0xAA, 0xBB, 0xCC, 0xDD};
    const auto good_frame = build_frame(0x0042u, body);
    feed_all(sync, good_frame, out);

    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].blockId(),             0x0042u);
    EXPECT_EQ(sync.stats().frames_emitted,  1u);
    EXPECT_EQ(sync.stats().length_invalid,  1u);  // unchanged
}

// TC7: Back-to-back frames in one feed — concatenate two valid frames.
// Both must be emitted in order with correct body contents.
TEST(FrameSync, BackToBackFramesOneFeed) {
    asterx::sbf::SbfFrameSync sync;
    std::vector<asterx::sbf::Frame> out;

    const std::vector<std::uint8_t> body1 = {0x01, 0x02, 0x03, 0x04,
                                              0x05, 0x06, 0x07, 0x08};
    const std::vector<std::uint8_t> body2 = {0xA1, 0xA2, 0xA3, 0xA4};

    const auto frame1 = build_frame(0x1111u, body1);
    const auto frame2 = build_frame(0x2222u, body2);

    std::vector<std::uint8_t> input;
    input.insert(input.end(), frame1.begin(), frame1.end());
    input.insert(input.end(), frame2.begin(), frame2.end());

    feed_all(sync, input, out);

    ASSERT_EQ(out.size(), 2u);

    EXPECT_EQ(out[0].blockId(), 0x1111u);
    EXPECT_EQ(out[0].body().size(), body1.size());
    for (std::size_t i = 0; i < body1.size(); ++i) {
        EXPECT_EQ(out[0].body()[i], body1[i]) << "frame1 body mismatch at " << i;
    }

    // raw_id 0x2222 = 0010_0010_0010_0010: blockRev=1, blockId=0x0222 (13-bit mask).
    EXPECT_EQ(out[1].rawId(),   0x2222u);
    EXPECT_EQ(out[1].blockId(), 0x0222u);
    EXPECT_EQ(out[1].blockRev(), 1u);
    EXPECT_EQ(out[1].body().size(), body2.size());
    for (std::size_t i = 0; i < body2.size(); ++i) {
        EXPECT_EQ(out[1].body()[i], body2[i]) << "frame2 body mismatch at " << i;
    }

    EXPECT_EQ(sync.stats().frames_emitted, 2u);
    EXPECT_EQ(sync.stats().crc_failed,     0u);
    EXPECT_EQ(sync.stats().length_invalid, 0u);
    EXPECT_EQ(sync.stats().resyncs,        0u);
}

// TC8: Single-byte feeds — feed the same valid frame one byte at a time.
// The result must be identical to feeding the whole buffer at once.
TEST(FrameSync, SingleByteFeeds) {
    // Baseline: feed in one shot.
    const std::vector<std::uint8_t> body = {
        0x10, 0x20, 0x30, 0x40,
        0x50, 0x60, 0x70, 0x80
    };
    const std::uint16_t block_id = 0x0FAAu;
    const auto frame_bytes = build_frame(block_id, body);

    // Reference: single feed.
    asterx::sbf::SbfFrameSync ref_sync;
    std::vector<asterx::sbf::Frame> ref_out;
    feed_all(ref_sync, frame_bytes, ref_out);
    ASSERT_EQ(ref_out.size(), 1u);

    // Copy body from reference while the span is still valid.
    std::vector<std::uint8_t> ref_body(ref_out[0].body().begin(),
                                       ref_out[0].body().end());
    const std::uint16_t ref_block_id = ref_out[0].blockId();
    const std::uint16_t ref_length   = ref_out[0].length();

    // Under test: single-byte feeds.
    asterx::sbf::SbfFrameSync byte_sync;
    std::vector<asterx::sbf::Frame> byte_out;

    for (std::size_t i = 0; i < frame_bytes.size(); ++i) {
        const std::uint8_t b = frame_bytes[i];
        byte_sync.feed(asterx::ConstByteSpan{&b, 1u}, byte_out);
    }

    ASSERT_EQ(byte_out.size(), 1u)
        << "Expected exactly one frame from single-byte feeds";

    // Copy body before next feed() (or end of scope).
    std::vector<std::uint8_t> byte_body(byte_out[0].body().begin(),
                                        byte_out[0].body().end());

    EXPECT_EQ(byte_out[0].blockId(), ref_block_id);
    EXPECT_EQ(byte_out[0].length(),  ref_length);
    EXPECT_EQ(byte_body, ref_body);

    EXPECT_EQ(byte_sync.stats().frames_emitted, 1u);
    EXPECT_EQ(byte_sync.stats().crc_failed,     0u);
    EXPECT_EQ(byte_sync.stats().resyncs,        0u);
    EXPECT_EQ(byte_sync.stats().bytes_consumed,
              static_cast<std::uint64_t>(frame_bytes.size()));
}

// TC9: Reset clears in-progress state but does NOT reset cumulative stats.
TEST(FrameSync, ResetClearsStateNotStats) {
    asterx::sbf::SbfFrameSync sync;
    std::vector<asterx::sbf::Frame> out;

    // Emit one frame to bump counters.
    const auto frame_bytes = build_frame(0x0001u);
    feed_all(sync, frame_bytes, out);
    ASSERT_EQ(out.size(), 1u);

    const auto stats_before = sync.stats();
    EXPECT_EQ(stats_before.frames_emitted, 1u);

    // Now feed the first two bytes of another frame to set state != Idle.
    const std::uint8_t partial[2] = {0x24u, 0x40u};
    out.clear();
    sync.feed(asterx::ConstByteSpan{partial, 2u}, out);
    EXPECT_EQ(out.size(), 0u);

    // reset() — clears in-progress state.
    sync.reset();

    // After reset, a fresh valid frame should parse cleanly.
    const auto frame2 = build_frame(0x0002u);
    feed_all(sync, frame2, out);

    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].blockId(), 0x0002u);

    // Stats continue accumulating (not reset by reset()).
    EXPECT_EQ(sync.stats().frames_emitted, 2u);
    EXPECT_GE(sync.stats().bytes_consumed, stats_before.bytes_consumed);
}

// TC10: Length-zero body (Length == 8) round-trip via single-byte feeds.
TEST(FrameSync, ZeroBodySingleByteFeeds) {
    asterx::sbf::SbfFrameSync sync;
    std::vector<asterx::sbf::Frame> out;

    const auto frame_bytes = build_frame(0x0777u);
    ASSERT_EQ(frame_bytes.size(), 8u);

    for (std::size_t i = 0; i < frame_bytes.size(); ++i) {
        const std::uint8_t b = frame_bytes[i];
        sync.feed(asterx::ConstByteSpan{&b, 1u}, out);
    }

    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].blockId(),     0x0777u);
    EXPECT_EQ(out[0].body().size(), 0u);
    EXPECT_EQ(sync.stats().frames_emitted, 1u);
}

// TC11: Frame with block revision bits set.
//
// The ID field is uint16 with low 13 bits = block_id and high 3 bits = rev.
// blockId() must strip the revision; blockRev() must return it.
TEST(FrameSync, BlockRevBitsStripped) {
    asterx::sbf::SbfFrameSync sync;
    std::vector<asterx::sbf::Frame> out;

    // raw_id with rev=3 (bits 13..15 = 0b011 = 0x6000), block_id = 0x1234.
    const std::uint16_t raw_id   = static_cast<std::uint16_t>(0x6000u | 0x1234u);
    const std::uint16_t block_id = raw_id & 0x1FFFu;  // = 0x1234
    const std::uint8_t  rev      = static_cast<std::uint8_t>(raw_id >> 13);  // = 3

    const std::vector<std::uint8_t> body = {0x01, 0x02, 0x03, 0x04};
    auto frame_bytes = build_frame(block_id, body);
    // build_frame puts block_id in [4..5]; raw_id has rev bits too.
    // We must overwrite bytes [4..5] with raw_id.
    frame_bytes[4] = static_cast<std::uint8_t>(raw_id & 0xFFu);
    frame_bytes[5] = static_cast<std::uint8_t>((raw_id >> 8) & 0xFFu);
    // Recompute CRC over bytes [4..Length-1].
    const std::uint16_t crc = asterx::sbf::crc_ccitt_xmodem(
        frame_bytes.data() + 4u, frame_bytes.size() - 4u);
    frame_bytes[2] = static_cast<std::uint8_t>(crc & 0xFFu);
    frame_bytes[3] = static_cast<std::uint8_t>((crc >> 8) & 0xFFu);

    feed_all(sync, frame_bytes, out);

    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].rawId(),   raw_id);
    EXPECT_EQ(out[0].blockId(), block_id);
    EXPECT_EQ(out[0].blockRev(), rev);
}

// TC12: kMaxSbfLength boundary — a frame with Length = kMaxSbfLength (8188)
// is accepted; a frame with Length = 8192 (> 8188) is rejected as
// length_invalid.
//
// We only test the rejection case here (building an 8188-byte frame would be
// expensive for a unit test).  We craft the invalid-length header manually.
TEST(FrameSync, LengthAtMaxRejectedAboveMax) {
    {
        // Length = 8192 (> 8188) — should be rejected.
        asterx::sbf::SbfFrameSync sync;
        std::vector<asterx::sbf::Frame> out;

        std::vector<std::uint8_t> bad = {
            0x24u, 0x40u,    // sync
            0x00u, 0x00u,    // CRC placeholder
            0x01u, 0x00u,    // ID = 1
            0x00u, 0x20u,    // Length = 0x2000 = 8192 (> 8188)
        };

        feed_all(sync, bad, out);

        EXPECT_EQ(out.size(),                  0u);
        EXPECT_EQ(sync.stats().length_invalid, 1u);
    }

    {
        // Length = 8 (minimum valid) — should be accepted.
        asterx::sbf::SbfFrameSync sync;
        std::vector<asterx::sbf::Frame> out;

        const auto frame = build_frame(0x0001u);  // Length == 8
        feed_all(sync, frame, out);

        EXPECT_EQ(out.size(), 1u);
        EXPECT_EQ(sync.stats().length_invalid, 0u);
    }
}

// TC13: Multiple garbage bytes containing embedded '$' characters.
//
// Ensures the GotDollar handler correctly rejects '$' followed by non-'@'
// and eventually finds the true '$@' sync.
TEST(FrameSync, GarbageWithEmbeddedDollars) {
    asterx::sbf::SbfFrameSync sync;
    std::vector<asterx::sbf::Frame> out;

    // Garbage: '$' '$' '$' followed by non-'@' bytes, then the real frame.
    const std::vector<std::uint8_t> body = {0xDE, 0xAD, 0xBE, 0xEF};
    const auto good_frame = build_frame(0x0333u, body);

    std::vector<std::uint8_t> input = {
        0x24u, 0x24u, 0x24u,  // three consecutive '$' (not '@' after any)
        0x01u, 0x02u           // non-'$' bytes
    };
    input.insert(input.end(), good_frame.begin(), good_frame.end());

    feed_all(sync, input, out);

    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].blockId(), 0x0333u);
    EXPECT_EQ(sync.stats().frames_emitted, 1u);
    EXPECT_EQ(sync.stats().crc_failed,     0u);
}

// TC14: CRC field check — verify that the Frame::crc() accessor returns the
// header CRC field exactly as stored (regardless of endianness).
TEST(FrameSync, FrameCrcAccessorMatchesHeader) {
    asterx::sbf::SbfFrameSync sync;
    std::vector<asterx::sbf::Frame> out;

    const std::vector<std::uint8_t> body = {0x01, 0x02, 0x03, 0x04};
    const auto frame_bytes = build_frame(0x0010u, body);

    // Read the expected CRC from the frame bytes.
    const std::uint16_t expected_crc = static_cast<std::uint16_t>(
        frame_bytes[2] | (static_cast<std::uint16_t>(frame_bytes[3]) << 8));

    feed_all(sync, frame_bytes, out);

    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].crc(), expected_crc);
}

// TC15: bytes_consumed accounts for ALL bytes, including junk.
TEST(FrameSync, BytesConsumedIncludesGarbage) {
    asterx::sbf::SbfFrameSync sync;
    std::vector<asterx::sbf::Frame> out;

    const std::size_t junk_len = 100u;
    const auto frame_bytes     = build_frame(0x0001u);
    const std::size_t total    = junk_len + frame_bytes.size();

    std::vector<std::uint8_t> input(junk_len, 0x55u);
    input.insert(input.end(), frame_bytes.begin(), frame_bytes.end());

    feed_all(sync, input, out);

    EXPECT_EQ(sync.stats().bytes_consumed, static_cast<std::uint64_t>(total));
}

// TC16: CRC over SBF spec-correct byte range — feed a hand-crafted frame
// where the CRC is computed only over [4..Length-1] (ID + Length + body),
// and verify the parser accepts it.  This is the "most common mistake" guard
// mentioned in §2.3 of the parser design doc.
TEST(FrameSync, CrcCoverageIsCorrectRange) {
    asterx::sbf::SbfFrameSync sync;
    std::vector<asterx::sbf::Frame> out;

    // Build a frame manually to have full visibility.
    //   sync[2] crc[2] id[2] len[2] body[4]   -- total 12 bytes
    const std::uint16_t block_id = 0x00ABu;
    const std::uint16_t length   = 12u;  // 8 header + 4 body
    const std::array<std::uint8_t, 4> body_arr = {0x11u, 0x22u, 0x33u, 0x44u};

    // Bytes [4..11]: id_lo id_hi len_lo len_hi body[0..3]
    std::array<std::uint8_t, 8> crc_data = {
        static_cast<std::uint8_t>(block_id & 0xFFu),
        static_cast<std::uint8_t>((block_id >> 8) & 0xFFu),
        static_cast<std::uint8_t>(length & 0xFFu),
        static_cast<std::uint8_t>((length >> 8) & 0xFFu),
        body_arr[0], body_arr[1], body_arr[2], body_arr[3]
    };
    const std::uint16_t crc = asterx::sbf::crc_ccitt_xmodem(
        crc_data.data(), crc_data.size());

    std::vector<std::uint8_t> frame = {
        0x24u, 0x40u,                                     // sync
        static_cast<std::uint8_t>(crc & 0xFFu),           // CRC lo
        static_cast<std::uint8_t>((crc >> 8) & 0xFFu),    // CRC hi
        crc_data[0], crc_data[1],                          // ID
        crc_data[2], crc_data[3],                          // Length
        body_arr[0], body_arr[1], body_arr[2], body_arr[3] // body
    };

    ASSERT_EQ(frame.size(), 12u);

    feed_all(sync, frame, out);

    ASSERT_EQ(out.size(), 1u)
        << "Expected one frame; CRC range may be wrong if 0 frames";
    EXPECT_EQ(out[0].blockId(), block_id);
    EXPECT_EQ(sync.stats().crc_failed, 0u);
}

// TC17: Regression for the 2026-05-25 long-soak heap corruption.
//
// Scenario that previously corrupted SbfFrameSync's internal std::deque:
//   feed #1 contained a complete valid frame A, followed by '$@' (the first
//   two bytes of a frame B).  The state machine emitted A (setting an
//   internal frame_drain_ counter to A's total length), then consumed '$@'
//   into ReadingHeader and stalled.  The end-of-feed cleanup branch erased
//   the prefix [0, frame_start) — which fully covered the frame_drain_
//   region — but did NOT reset frame_drain_.  On the next feed() the drain
//   step then tried to erase that many bytes again, walking _M_finish past
//   _M_start and leaving the deque in a state that surfaced minutes later
//   as a glibc "double free detected in tcache 2" abort.
//
// With the fix, frame_drain_ is reset to 0 alongside the erase, so the
// second feed() proceeds normally and emits frame B.
TEST(FrameSync, EmitThenStallDoesNotOverDrain) {
    asterx::sbf::SbfFrameSync sync;
    std::vector<asterx::sbf::Frame> out;

    // Frame A: 24 bytes (8 header + 16 body).
    const std::vector<std::uint8_t> body_a(16u, 0xAAu);
    const auto frame_a = build_frame(0x1001u, body_a);
    ASSERT_EQ(frame_a.size(), 24u);

    // Frame B: 12 bytes (8 header + 4 body).
    const std::vector<std::uint8_t> body_b = {0xB1, 0xB2, 0xB3, 0xB4};
    const auto frame_b = build_frame(0x1002u, body_b);
    ASSERT_EQ(frame_b.size(), 12u);

    // Feed #1: complete frame A + first 2 bytes of frame B ('$', '@').
    std::vector<std::uint8_t> first_feed(frame_a.begin(), frame_a.end());
    first_feed.push_back(frame_b[0]);  // '$'
    first_feed.push_back(frame_b[1]);  // '@'
    feed_all(sync, first_feed, out);

    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].blockId(),            0x1001u);
    EXPECT_EQ(sync.stats().frames_emitted, 1u);
    EXPECT_EQ(sync.stats().crc_failed,     0u);
    EXPECT_EQ(sync.stats().length_invalid, 0u);

    // Feed #2: the remaining 10 bytes of frame B.
    // Before the fix, the drain step at entry would erase 24 bytes from a
    // 2-byte deque (UB).  With the fix, drain is skipped and frame B parses
    // cleanly.
    out.clear();
    std::vector<std::uint8_t> second_feed(frame_b.begin() + 2, frame_b.end());
    feed_all(sync, second_feed, out);

    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].blockId(),            0x1002u);
    EXPECT_EQ(sync.stats().frames_emitted, 2u);
    EXPECT_EQ(sync.stats().crc_failed,     0u);
    EXPECT_EQ(sync.stats().length_invalid, 0u);
    EXPECT_EQ(sync.stats().resyncs,        0u);

    ASSERT_EQ(out[0].body().size(), body_b.size());
    for (std::size_t i = 0; i < body_b.size(); ++i) {
        EXPECT_EQ(out[0].body()[i], body_b[i]) << "body B mismatch at " << i;
    }

    // Repeat the same pattern many times to amplify any latent deque
    // corruption (the original bug only surfaced after tens of thousands of
    // frames in a long-running session).
    for (int iter = 0; iter < 200; ++iter) {
        out.clear();
        std::vector<std::uint8_t> a_plus_partial(frame_a.begin(), frame_a.end());
        a_plus_partial.push_back(frame_b[0]);
        a_plus_partial.push_back(frame_b[1]);
        feed_all(sync, a_plus_partial, out);
        ASSERT_EQ(out.size(), 1u) << "iter=" << iter << ": A not emitted";

        out.clear();
        std::vector<std::uint8_t> rest(frame_b.begin() + 2, frame_b.end());
        feed_all(sync, rest, out);
        ASSERT_EQ(out.size(), 1u) << "iter=" << iter << ": B not emitted";
        EXPECT_EQ(out[0].blockId(), 0x1002u) << "iter=" << iter;
    }

    EXPECT_EQ(sync.stats().frames_emitted, 2u + 200u * 2u);
    EXPECT_EQ(sync.stats().crc_failed,     0u);
    EXPECT_EQ(sync.stats().length_invalid, 0u);
    EXPECT_EQ(sync.stats().resyncs,        0u);
}

}  // namespace
