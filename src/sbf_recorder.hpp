// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

#include "sbf/frame_sync.hpp"

namespace asterx {

struct RecorderStats {
    std::uint64_t bytes_written {0};
    std::uint64_t frames_written{0};
    std::uint64_t files_rotated {0};
    std::uint64_t crc_failed    {0};
    std::uint64_t length_invalid{0};
    std::uint64_t resyncs       {0};
};

// Streams raw SBF frames to disk, one file at a time, rotating when the
// current file passes a size or wall-time threshold.
//
// Output filenames: <prefix>-YYYYMMDDTHHMMSSZ-<seq>.sbf  (seq starts at 1).
//
// Each frame written to disk is reconstructed from its FrameSync-emitted
// fields (sync 0x24 0x40, CRC, ID, Length, body) and is therefore
// byte-identical to what arrived on the wire after CRC verification.
class SbfRecorder {
public:
    struct Config {
        std::filesystem::path output_dir{"./recordings"};
        std::string           file_prefix{"asterx"};
        std::uint64_t         rotate_bytes{1ull << 30};                   // 1 GiB
        std::chrono::seconds  rotate_interval{std::chrono::hours(1)};
    };

    explicit SbfRecorder(Config cfg);
    ~SbfRecorder();

    SbfRecorder(const SbfRecorder&)            = delete;
    SbfRecorder& operator=(const SbfRecorder&) = delete;

    // Feed bytes received from the receiver. Frames are validated, then
    // written verbatim to the current file. Rotation is checked after each
    // write.  Throws std::runtime_error on disk I/O failure.
    void feed(asterx::ConstByteSpan bytes);

    // Flush + close the current file.
    void close() noexcept;

    [[nodiscard]] const RecorderStats& stats() const noexcept { return stats_; }
    [[nodiscard]] std::filesystem::path current_file() const noexcept { return current_path_; }

private:
    void open_new_file_();
    void write_frame_(const sbf::Frame& frame);
    void rotate_if_needed_();

    Config                       cfg_;
    sbf::SbfFrameSync            sync_;
    std::vector<sbf::Frame>      out_;     // reusable scratch
    std::uint64_t                prev_resyncs_       {0};
    std::uint64_t                prev_crc_failed_    {0};
    std::uint64_t                prev_length_invalid_{0};

    std::FILE*                   fp_{nullptr};
    std::filesystem::path        current_path_;
    std::uint64_t                current_size_{0};
    std::chrono::steady_clock::time_point file_start_{};
    int                          seq_{0};

    RecorderStats                stats_{};
};

}  // namespace asterx
