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
    std::string              interval;    // e.g. "OnChange"
};

struct Vec3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

struct AttitudeOffset {
    double heading_deg{0.0};
    double pitch_deg{0.0};
};

struct ReceiverCapabilities {
    bool has_main{false};
    bool has_aux1{false};
    int measurement_interval_ms{0};
    int pvt_interval_ms{0};
    int ins_interval_ms{0};
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
        {1, {"ExtSensorMeas"},                                       "OnChange"},
        {2, {"MeasEpoch","MeasExtra","EndOfMeas"},                   "OnChange"},
        {3, {"AuxAntPositions","AttEuler","AttCovEuler","EndOfAtt"}, "OnChange"},
        {4, {"ExtSensorStatus","ExtSensorInfo","IMUSetup"},          "OnChange"},
        {5, {"ReceiverStatus","QualityInd","ReceiverTime","ChannelStatus"}, "OnChange"},
    };

    // Legacy config compatibility: load_config maps this to imu_orientation_mode.
    bool   use_sensor_default{true};

    // IMU setup. OrientationMode is SensorDefault, manual, or fixed.
    std::string imu_startup_data_mode{"Boot"};
    std::string imu_orientation_mode{"SensorDefault"};
    double theta_x_deg{0.0};
    double theta_y_deg{0.0};
    double theta_z_deg{0.0};
    Vec3   ant_lever_arm_m{};
    bool   ant_lever_arm_configured{false};

    // Dual-antenna GNSS attitude.
    bool require_aux1{true};
    std::string gnss_attitude_mode{"MultiAntenna"};
    AttitudeOffset attitude_offset_deg{};
};

[[nodiscard]] bool is_valid_sbf_interval(const std::string& interval);
[[nodiscard]] std::string build_sbf_output_command(const SbfStream& stream, int ips_id);
[[nodiscard]] std::string build_ins_ant_lever_arm_command(Vec3 lever_arm_m);
[[nodiscard]] ReceiverCapabilities parse_receiver_capabilities_reply(const std::string& reply);
void validate_receiver_settings(const ReceiverSettings& settings);
void verify_imu_orientation_reply(const std::string& reply, const ReceiverSettings& settings);
void verify_ins_ant_lever_arm_reply(const std::string& reply, Vec3 expected);
void verify_gnss_attitude_reply(const std::string& reply, const std::string& expected_mode);
void verify_attitude_offset_reply(const std::string& reply, AttitudeOffset expected);

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

    // Apply all settings: capabilities check, IMU/lever-arm/attitude setup,
    // IPS output setup, and setSBFOutput Stream*.
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
