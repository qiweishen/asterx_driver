// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "tcp_client.hpp"

namespace asterx {

class ConfigError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// One SBF stream definition pushed to the receiver. Each stream lists a set
// of SBF blocks and a single MsgInterval token (e.g. "msec5", "msec100", "sec1").
struct SbfStream {
    int                      stream_id;   // 1..10 — receiver-side Stream<id>
    std::vector<std::string> blocks;      // e.g. {"ExtSensorMeas"}
    std::string              interval;    // e.g. "msec5"
};

struct ReceiverSettings {
    // Login
    std::string user{"admin"};
    std::string password{"admin"};

    // IPS data port — where the receiver pushes SBF bytes once configured.
    int           ips_id  {1};          // IPS1..IPS5
    std::uint16_t ips_port{28785};      // listen port on the receiver

    // SBF stream layout (defaults match docs/architecture/00-requirements.md Q12).
    // EndOfMeas / EndOfAtt are epoch terminators that help downstream parsers
    // know when a measurement / attitude epoch is complete.
    std::vector<SbfStream> streams{
        {1, {"ExtSensorMeas"},                                       "msec5"  },
        {2, {"MeasEpoch","MeasExtra","EndOfMeas"},                   "msec100"},
        {3, {"AuxAntPositions","AttEuler","AttCovEuler","EndOfAtt"}, "msec100"},
        {4, {"ReceiverStatus","QualityInd","ReceiverTime"},          "sec1"   },
    };

    // IMU orientation (Tait-Bryan degrees, receiver → IMU body). If
    // use_sensor_default is true, "SensorDefault" is pushed regardless of theta.
    bool   use_sensor_default{true};
    double theta_x_deg{0.0};
    double theta_y_deg{0.0};
    double theta_z_deg{0.0};
};

// Drives the control connection through the canonical configure-and-launch
// sequence. Pure command builder + reply parser; does not own the recording.
class ReceiverConfigurator {
public:
    explicit ReceiverConfigurator(TcpClient& ctrl) noexcept : ctrl_(ctrl) {}

    // 1) Read the receiver banner (a few lines ending with "USER>" or similar).
    // 2) Send "login, <user>, <pass>".
    // 3) Verify the prompt becomes the post-login form.
    void login(const std::string& user,
               const std::string& password,
               std::chrono::milliseconds timeout = std::chrono::seconds(5));

    // Apply all settings: setIPServerSettings, setSBFOutput Stream*, setImuOrientation.
    // Throws ConfigError if the receiver returns "?" / error replies.
    void apply(const ReceiverSettings& settings,
               std::chrono::milliseconds per_cmd_timeout = std::chrono::seconds(3));

    // Best-effort: tell the receiver to stop all SBF streams on the IPS1 port.
    // Used on shutdown. Never throws.
    void stop_streams(const ReceiverSettings& settings) noexcept;

private:
    // Send a single command line (CR terminated), read until the next prompt,
    // throw ConfigError if the reply contains "$R? " (error prefix).
    std::string send_command(const std::string& cmd,
                             std::chrono::milliseconds timeout);

    TcpClient& ctrl_;
    std::string prompt_{">"};  // refined after login
};

}  // namespace asterx
