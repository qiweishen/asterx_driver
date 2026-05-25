// SPDX-License-Identifier: BSD-3-Clause
//
// asterx_driver — entry point.
//
//   ./asterx_driver --config config.yaml
//
// Single-threaded recorder: open control socket, push receiver config,
// open data socket, read SBF bytes, validate + write to a rotating .sbf file.
//
#include <atomic>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <yaml-cpp/yaml.h>

#include "tcp_client.hpp"
#include "receiver_config.hpp"
#include "sbf_recorder.hpp"

namespace {
    std::atomic<bool> g_stop{false};

    void on_signal(int /*sig*/) { g_stop.store(true, std::memory_order_relaxed); }

    struct AppConfig {
        // [connection]
        std::string host{"192.168.3.1"};
        std::uint16_t ctrl_port{28784};
        std::string user{"admin"};
        std::string password{"admin"};

        // [output]
        std::filesystem::path output_dir{"./recordings"};
        std::string file_prefix{"asterx"};
        std::uint64_t rotate_bytes{1ull << 30};
        int rotate_interval_seconds{3600};

        // [log]
        std::filesystem::path log_dir{"./logs"};
        std::string log_level{"info"};
        std::uint64_t log_max_size{100ull * 1024 * 1024};
        int log_max_files{5};

        // [receiver]
        asterx::ReceiverSettings receiver{};
    };

    [[noreturn]] void die(const std::string &msg, int code = 2) {
        std::cerr << "asterx_driver: " << msg << "\n";
        std::exit(code);
    }

    AppConfig load_config(const std::string &path) {
        AppConfig c;
        YAML::Node root;
        try {
            root = YAML::LoadFile(path);
        } catch (const std::exception &e) {
            die(std::string("failed to load config '") + path + "': " + e.what());
        }

        if (auto n = root["connection"]) {
            if (n["host"]) c.host = n["host"].as<std::string>();
            if (n["port"]) c.ctrl_port = static_cast<std::uint16_t>(n["port"].as<int>());
            if (n["user"]) c.user = n["user"].as<std::string>();
            if (n["password"]) c.password = n["password"].as<std::string>();
        }
        if (auto n = root["output"]) {
            if (n["dir"]) c.output_dir = n["dir"].as<std::string>();
            if (n["file_prefix"]) c.file_prefix = n["file_prefix"].as<std::string>();
            if (n["rotate_bytes"]) c.rotate_bytes = n["rotate_bytes"].as<std::uint64_t>();
            if (n["rotate_interval_s"]) c.rotate_interval_seconds = n["rotate_interval_s"].as<int>();
        }
        if (auto n = root["log"]) {
            if (n["dir"]) c.log_dir = n["dir"].as<std::string>();
            if (n["level"]) c.log_level = n["level"].as<std::string>();
            if (n["max_size"]) c.log_max_size = n["max_size"].as<std::uint64_t>();
            if (n["max_files"]) c.log_max_files = n["max_files"].as<int>();
        }
        if (auto n = root["receiver"]) {
            if (n["ips_id"]) c.receiver.ips_id = n["ips_id"].as<int>();
            if (n["ips_port"]) c.receiver.ips_port = static_cast<std::uint16_t>(n["ips_port"].as<int>());
            if (n["imu"]) {
                auto im = n["imu"];
                if (im["use_sensor_default"]) c.receiver.use_sensor_default = im["use_sensor_default"].as<bool>();
                if (im["theta_x_deg"]) c.receiver.theta_x_deg = im["theta_x_deg"].as<double>();
                if (im["theta_y_deg"]) c.receiver.theta_y_deg = im["theta_y_deg"].as<double>();
                if (im["theta_z_deg"]) c.receiver.theta_z_deg = im["theta_z_deg"].as<double>();
            }
            if (n["streams"] && n["streams"].IsSequence()) {
                c.receiver.streams.clear();
                for (const auto &s: n["streams"]) {
                    asterx::SbfStream st;
                    st.stream_id = s["id"].as<int>();
                    st.interval = s["interval"].as<std::string>();
                    for (const auto &b: s["blocks"]) st.blocks.push_back(b.as<std::string>());
                    c.receiver.streams.push_back(std::move(st));
                }
            }
        }
        c.receiver.user = c.user;
        c.receiver.password = c.password;
        return c;
    }

    void setup_logging(const AppConfig &cfg) {
        std::error_code ec;
        std::filesystem::create_directories(cfg.log_dir, ec);

        auto stderr_sink = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
        auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            (cfg.log_dir / "asterx.log").string(),
            cfg.log_max_size,
            static_cast<std::size_t>(cfg.log_max_files));

        auto logger = std::make_shared<spdlog::logger>(
            "asterx",
            spdlog::sinks_init_list{stderr_sink, file_sink});
        logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");
        spdlog::set_default_logger(logger);
        spdlog::set_level(spdlog::level::from_str(cfg.log_level));
        spdlog::flush_on(spdlog::level::warn);
    }

    void print_usage() {
        std::cerr <<
                "Usage: asterx_driver --config <path.yaml> [--log-level <level>]\n"
                "  --config <path>        YAML config file (see config.example.yaml)\n"
                "  --log-level <level>    trace|debug|info|warn|err|critical (overrides config)\n"
                "  --help                 print this message\n";
    }
} // namespace

int main(int argc, char **argv) {
    std::string config_path;
    std::string log_level_override;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--help" || a == "-h") {
            print_usage();
            return 0;
        } else if (a == "--config" && i + 1 < argc) { config_path = argv[++i]; } else if (
            a == "--log-level" && i + 1 < argc) { log_level_override = argv[++i]; } else {
            print_usage();
            return 3;
        }
    }
    if (config_path.empty()) {
        print_usage();
        return 3;
    }

    AppConfig cfg = load_config(config_path);
    if (!log_level_override.empty()) {
        cfg.log_level = log_level_override;
    }
    setup_logging(cfg);

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    spdlog::info("asterx_driver starting (host={}, ctrl_port={}, ips{}_port={})",
                 cfg.host, cfg.ctrl_port, cfg.receiver.ips_id, cfg.receiver.ips_port);

    asterx::TcpClient ctrl;
    asterx::TcpClient data;

    try {
        // 1) Control channel: connect, log in, push config.
        ctrl.connect(cfg.host, cfg.ctrl_port, std::chrono::seconds(5));
        asterx::ReceiverConfigurator rc(ctrl);
        rc.login(cfg.user, cfg.password);
        rc.apply(cfg.receiver);

        // 2) Data channel: connect to the receiver-side IPS port.
        data.connect(cfg.host, cfg.receiver.ips_port, std::chrono::seconds(5));
        spdlog::info("data channel open on {}:{}", cfg.host, cfg.receiver.ips_port);

        // 3) Recorder loop.
        asterx::SbfRecorder rec(asterx::SbfRecorder::Config{
            cfg.output_dir,
            cfg.file_prefix,
            cfg.rotate_bytes,
            std::chrono::seconds(cfg.rotate_interval_seconds),
        });

        std::uint8_t buf[65536];
        auto last_log = std::chrono::steady_clock::now();
        while (!g_stop.load(std::memory_order_relaxed)) {
            try {
                std::size_t n = data.read_some(
                    asterx::MutByteSpan{buf, sizeof(buf)},
                    std::chrono::seconds(10));
                if (n == 0) {
                    spdlog::warn("data channel closed by peer");
                    break;
                }
                rec.feed(asterx::ConstByteSpan{buf, n});
            } catch (const asterx::TcpError &e) {
                spdlog::error("read error: {}", e.what());
                break;
            }

            const auto now = std::chrono::steady_clock::now();
            if (now - last_log >= std::chrono::seconds(5)) {
                const auto &s = rec.stats();
                spdlog::info("recorder: frames={} bytes={} files={} crc_fail={} resyncs={}",
                             s.frames_written, s.bytes_written, s.files_rotated,
                             s.crc_failed, s.resyncs);
                last_log = now;
            }
        }

        spdlog::info("stopping; final stats: frames={} bytes={} files={} crc_fail={} resyncs={}",
                     rec.stats().frames_written, rec.stats().bytes_written,
                     rec.stats().files_rotated, rec.stats().crc_failed, rec.stats().resyncs);

        rec.close();
        rc.stop_streams(cfg.receiver);
    } catch (const std::exception &e) {
        spdlog::critical("fatal: {}", e.what());
        return 1;
    }

    spdlog::info("clean shutdown");
    return 0;
}
