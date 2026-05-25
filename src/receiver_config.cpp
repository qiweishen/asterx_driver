// SPDX-License-Identifier: BSD-3-Clause
//
// Implements the small subset of the Septentrio ASCII command interface that
// the slim driver needs:
//
//   login, <user>, <pass>
//   setIPServerSettings, IPS<n>, <port>, TCP
//   setDataInOut, IPS<n>, , +SBF      (enable SBF output on the IPS channel)
//   setSBFOutput, Stream<id>, IPS<n>, <block+block+...>, <interval>
//   setImuOrientation, SensorDefault | manual, <θX>, <θY>, <θZ>
//
// Replies follow the convention:
//
//   $R; <echo of command>\r\n
//   $R? <error_text>\r\n           on error
//
// Followed by a prompt line ending with ">". We send one command at a time
// and read until the next ">" character is observed.
//
// References:
//   docs/architecture/03-sbf-parser-design.md
//   .claude/skills/septentrio-receiver-config/SKILL.md
//   docs/AsteRx-i3_D_Pro__Firmware_v1_5_2_Reference_Guide.pdf
//
#include "receiver_config.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <sstream>

#include <spdlog/spdlog.h>

namespace asterx {

namespace {

std::string join(const std::vector<std::string>& parts, char sep) {
    std::string out;
    for (std::size_t i = 0; i < parts.size(); ++i) {
        if (i) out.push_back(sep);
        out.append(parts[i]);
    }
    return out;
}

// Redact "login, user, password" lines before logging.
std::string redact_cmd(const std::string& cmd) {
    if (cmd.rfind("login", 0) == 0) return "login, <REDACTED>, <REDACTED>";
    return cmd;
}

}  // namespace

std::string
ReceiverConfigurator::send_command(const std::string& cmd,
                                   std::chrono::milliseconds timeout) {
    spdlog::debug("[receiver] -> {}", redact_cmd(cmd));
    ctrl_.write_all(cmd + "\r\n");

    // Receivers reply with a body and end with a "PROMPT>" line. Reading
    // until ">" is sufficient because all command echoes terminate with the
    // prompt.
    std::string reply = ctrl_.read_until(">", timeout);
    spdlog::debug("[receiver] <- {} bytes", reply.size());

    // "$R?" prefix or "USAGE:" anywhere in the reply indicates an error.
    if (reply.find("$R?") != std::string::npos ||
        reply.find("Invalid") != std::string::npos) {
        throw ConfigError("receiver rejected '" + redact_cmd(cmd) + "': " + reply);
    }
    return reply;
}

void ReceiverConfigurator::login(const std::string& user,
                                 const std::string& password,
                                 std::chrono::milliseconds timeout) {
    // Drain the banner. Receivers emit a few lines + a "USER>" prompt on
    // first connect. We read until any prompt character.
    std::string banner = ctrl_.read_until(">", timeout);
    spdlog::info("[receiver] banner: {} bytes", banner.size());

    const std::string cmd = "login, " + user + ", " + password;
    std::string reply = send_command(cmd, timeout);

    if (reply.find("$R?") != std::string::npos) {
        throw ConfigError("login refused (check credentials)");
    }
    spdlog::info("[receiver] login OK as user={}", user);
}

void ReceiverConfigurator::apply(const ReceiverSettings& settings,
                                 std::chrono::milliseconds t) {
    // 1) Wipe existing SBF output on the streams we're about to use, so a
    //    stale config doesn't double-send.
    for (const auto& s : settings.streams) {
        char buf[128];
        std::snprintf(buf, sizeof(buf),
                      "setSBFOutput, Stream%d, none, none, off", s.stream_id);
        try { send_command(buf, t); }
        catch (const ConfigError& e) {
            // Receiver may complain "Stream<id> not configured" — that's fine.
            spdlog::debug("[receiver] clear-stream{}: {}", s.stream_id, e.what());
        }
    }

    // 2) IPS data port (TCP server side on the receiver).
    {
        char buf[128];
        std::snprintf(buf, sizeof(buf),
                      "setIPServerSettings, IPS%d, %u, TCP",
                      settings.ips_id, settings.ips_port);
        send_command(buf, t);
    }

    // 3) Enable SBF output on the IPS channel. Without this the receiver
    //    opens the TCP listener but never pushes any bytes. The empty middle
    //    field leaves the input direction unchanged.
    {
        char buf[64];
        std::snprintf(buf, sizeof(buf),
                      "setDataInOut, IPS%d, , +SBF", settings.ips_id);
        send_command(buf, t);
    }

    // 4) IMU orientation.
    if (settings.use_sensor_default) {
        send_command("setImuOrientation, SensorDefault", t);
    } else {
        char buf[160];
        std::snprintf(buf, sizeof(buf),
                      "setImuOrientation, manual, %.6f, %.6f, %.6f",
                      settings.theta_x_deg, settings.theta_y_deg, settings.theta_z_deg);
        send_command(buf, t);
    }

    // 5) SBF stream membership + intervals.
    for (const auto& s : settings.streams) {
        if (s.blocks.empty()) continue;
        std::string blocks_csv = join(s.blocks, '+');
        char buf[256];
        std::snprintf(buf, sizeof(buf),
                      "setSBFOutput, Stream%d, IPS%d, %s, %s",
                      s.stream_id, settings.ips_id,
                      blocks_csv.c_str(), s.interval.c_str());
        send_command(buf, t);
        spdlog::info("[receiver] Stream{} ({} @ {})",
                     s.stream_id, blocks_csv, s.interval);
    }
}

void ReceiverConfigurator::stop_streams(const ReceiverSettings& settings) noexcept {
    for (const auto& s : settings.streams) {
        char buf[128];
        std::snprintf(buf, sizeof(buf),
                      "setSBFOutput, Stream%d, none, none, off", s.stream_id);
        try { send_command(buf, std::chrono::seconds(2)); }
        catch (...) { /* best-effort; the recorder is shutting down */ }
    }
}

}  // namespace asterx
