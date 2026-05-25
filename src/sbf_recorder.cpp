// SPDX-License-Identifier: BSD-3-Clause
#include "sbf_recorder.hpp"

#include <chrono>
#include <ctime>
#include <cstring>
#include <stdexcept>
#include <system_error>

#include <spdlog/spdlog.h>

namespace asterx {

namespace {

std::string utc_timestamp_now() {
    const auto t  = std::chrono::system_clock::now();
    const auto tt = std::chrono::system_clock::to_time_t(t);
    std::tm tm{};
    gmtime_r(&tt, &tm);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%dT%H%M%SZ", &tm);
    return std::string(buf);
}

}  // namespace

SbfRecorder::SbfRecorder(Config cfg) : cfg_(std::move(cfg)) {
    std::error_code ec;
    std::filesystem::create_directories(cfg_.output_dir, ec);
    if (ec) {
        throw std::runtime_error("cannot create output directory '" +
                                 cfg_.output_dir.string() + "': " + ec.message());
    }
    out_.reserve(64);
}

SbfRecorder::~SbfRecorder() { close(); }

void SbfRecorder::close() noexcept {
    if (fp_) {
        std::fflush(fp_);
        std::fclose(fp_);
        fp_           = nullptr;
        current_size_ = 0;
    }
}

void SbfRecorder::open_new_file_() {
    if (fp_) {
        std::fflush(fp_);
        std::fclose(fp_);
        fp_ = nullptr;
        ++stats_.files_rotated;
    }
    ++seq_;
    current_path_ = cfg_.output_dir /
                    (cfg_.file_prefix + "-" + utc_timestamp_now() + "-" +
                     std::to_string(seq_) + ".sbf");
    fp_ = std::fopen(current_path_.string().c_str(), "wb");
    if (!fp_) {
        throw std::runtime_error("cannot open output file '" +
                                 current_path_.string() +
                                 "': " + std::strerror(errno));
    }
    current_size_ = 0;
    file_start_   = std::chrono::steady_clock::now();
    spdlog::info("[recorder] opened {}", current_path_.string());
}

void SbfRecorder::rotate_if_needed_() {
    if (!fp_) return;
    const auto now    = std::chrono::steady_clock::now();
    const bool by_size = current_size_ >= cfg_.rotate_bytes;
    const bool by_time = (now - file_start_) >= cfg_.rotate_interval;
    if (by_size || by_time) {
        spdlog::info("[recorder] rotating ({} bytes, {} s)",
                     current_size_,
                     std::chrono::duration_cast<std::chrono::seconds>(now - file_start_).count());
        open_new_file_();
    }
}

void SbfRecorder::write_frame_(const sbf::Frame& frame) {
    // Reconstruct the on-wire header.
    const std::uint16_t crc    = frame.crc();
    const std::uint16_t raw_id = frame.rawId();
    const std::uint16_t length = frame.length();

    std::uint8_t header[8];
    header[0] = 0x24;  // '$'
    header[1] = 0x40;  // '@'
    header[2] = static_cast<std::uint8_t>(crc & 0xFFu);
    header[3] = static_cast<std::uint8_t>((crc >> 8) & 0xFFu);
    header[4] = static_cast<std::uint8_t>(raw_id & 0xFFu);
    header[5] = static_cast<std::uint8_t>((raw_id >> 8) & 0xFFu);
    header[6] = static_cast<std::uint8_t>(length & 0xFFu);
    header[7] = static_cast<std::uint8_t>((length >> 8) & 0xFFu);

    if (std::fwrite(header, 1, 8, fp_) != 8) {
        throw std::runtime_error("fwrite(header) failed");
    }
    const auto body = frame.body();
    if (!body.empty()) {
        if (std::fwrite(body.data(), 1, body.size(), fp_) != body.size()) {
            throw std::runtime_error("fwrite(body) failed");
        }
    }
    current_size_         += length;
    stats_.bytes_written  += length;
    ++stats_.frames_written;
}

void SbfRecorder::feed(asterx::ConstByteSpan bytes) {
    if (!fp_) open_new_file_();

    out_.clear();
    sync_.feed(bytes, out_);

    // Persist FrameSync's drop counters as recorder stats (deltas).
    const auto& fs = sync_.stats();
    stats_.crc_failed     += (fs.crc_failed     - prev_crc_failed_);
    stats_.length_invalid += (fs.length_invalid - prev_length_invalid_);
    stats_.resyncs        += (fs.resyncs        - prev_resyncs_);
    prev_crc_failed_      = fs.crc_failed;
    prev_length_invalid_  = fs.length_invalid;
    prev_resyncs_         = fs.resyncs;

    for (const auto& f : out_) {
        write_frame_(f);
        rotate_if_needed_();
    }
}

}  // namespace asterx
