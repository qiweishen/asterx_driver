// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <string>

#include "receiver_config.hpp"

namespace asterx {

class ConfigLoadError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

struct AppConfig {
    std::string host{"192.168.3.1"};
    std::uint16_t ctrl_port{28784};
    std::string user{"admin"};
    std::string password{"admin"};

    std::filesystem::path output_dir{"./recordings"};
    std::string file_prefix{"asterx"};
    std::uint64_t rotate_bytes{1ull << 30};
    int rotate_interval_seconds{3600};

    std::filesystem::path log_dir{"./logs"};
    std::string log_level{"info"};
    std::uint64_t log_max_size{100ull * 1024 * 1024};
    int log_max_files{5};

    ReceiverSettings receiver{};
};

[[nodiscard]] AppConfig load_app_config(const std::string& path);

}  // namespace asterx
