// SPDX-License-Identifier: BSD-3-Clause
//
// Implements the small subset of the Septentrio ASCII command interface that
// the slim driver needs:
//
//   login, <user>, <pass>
//   setIPServerSettings, IPS<n>, <port>, TCP
//   setDataInOut, IPS<n>, , +SBF      (enable SBF output on the IPS channel)
//   setSBFOutput, Stream<id>, IPS<n>, <block+block+...>, <interval>
//   setIMUStartupDataMode, Boot | GnssTimeKnown
//   setIMUOrientation, SensorDefault | manual | fixed, <thetaX>, <thetaY>, <thetaZ>
//   setINSAntLeverArm, <x>, <y>, <z>
//   setGNSSAttitude, MultiAntenna
//   setAttitudeOffset, <heading>, <pitch>
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
#include <cmath>
#include <iomanip>
#include <sstream>
#include <unordered_set>

#include <spdlog/spdlog.h>

namespace asterx {
    namespace {
        constexpr std::size_t kMaxAsciiCommandLength = 2000;

        std::string join(const std::vector<std::string> &parts, char sep) {
            std::string out;
            for (std::size_t i = 0; i < parts.size(); ++i) {
                if (i) out.push_back(sep);
                out.append(parts[i]);
            }
            return out;
        }

        std::string trim(std::string s) {
            auto not_space = [](unsigned char c) { return !std::isspace(c); };
            s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
            s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
            return s;
        }

        std::string lower_copy(std::string s) {
            std::transform(s.begin(), s.end(), s.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            return s;
        }

        bool equals_ci(const std::string &a, const std::string &b) {
            return lower_copy(a) == lower_copy(b);
        }

        bool starts_with_ci(const std::string &s, const std::string &prefix) {
            if (s.size() < prefix.size()) return false;
            return lower_copy(s.substr(0, prefix.size())) == lower_copy(prefix);
        }

        std::vector<std::string> split(const std::string &s, char sep) {
            std::vector<std::string> out;
            std::stringstream ss(s);
            std::string part;
            while (std::getline(ss, part, sep)) {
                out.push_back(trim(part));
            }
            return out;
        }

        std::string fmt6(double v) {
            std::ostringstream os;
            os << std::fixed << std::setprecision(6) << v;
            return os.str();
        }

        int parse_int_prefix(const std::string &s, const std::string &field) {
            std::istringstream is(trim(s));
            int v = 0;
            if (!(is >> v)) {
                throw ConfigError("could not parse integer field '" + field + "' from '" + s + "'");
            }
            return v;
        }

        double parse_double_prefix(const std::string &s, const std::string &field) {
            std::istringstream is(trim(s));
            double v = 0.0;
            if (!(is >> v)) {
                throw ConfigError("could not parse numeric field '" + field + "' from '" + s + "'");
            }
            return v;
        }

        bool close_enough(double a, double b) {
            return std::fabs(a - b) <= 0.001;
        }

        std::string find_config_line_payload(const std::string &reply, const std::string &key) {
            std::istringstream lines(reply);
            std::string line;
            const std::string prefix = key + ",";
            while (std::getline(lines, line)) {
                line = trim(line);
                if (starts_with_ci(line, prefix)) {
                    return trim(line.substr(prefix.size()));
                }
            }
            throw ConfigError("receiver reply did not contain '" + key + "' line: " + reply);
        }

        std::string compact_payload_after_key(const std::string &reply, const std::string &key) {
            const std::string marker = key + ",";
            const auto pos = reply.find(marker);
            if (pos == std::string::npos) {
                throw ConfigError("receiver reply did not contain '" + key + "' payload: " + reply);
            }

            std::string payload = reply.substr(pos + marker.size());
            const auto prompt = payload.find('>');
            if (prompt != std::string::npos) {
                payload = payload.substr(0, prompt);
            }
            for (char &c: payload) {
                if (c == '\r' || c == '\n' || c == '\t') c = ' ';
            }
            return payload;
        }

        void enforce_command_length(const std::string &cmd) {
            if (cmd.size() > kMaxAsciiCommandLength) {
                throw ConfigError("ASCII command exceeds Septentrio 2000-character limit");
            }
        }

        std::string set_orientation_command(const ReceiverSettings &settings) {
            if (equals_ci(settings.imu_orientation_mode, "SensorDefault")) {
                return "setIMUOrientation, SensorDefault";
            }
            return "setIMUOrientation, " + settings.imu_orientation_mode + ", " +
                   fmt6(settings.theta_x_deg) + ", " + fmt6(settings.theta_y_deg) + ", " +
                   fmt6(settings.theta_z_deg);
        }

        std::string set_vec3_command(const std::string &name, Vec3 v) {
            return name + ", " + fmt6(v.x) + ", " + fmt6(v.y) + ", " + fmt6(v.z);
        }

        // Redact "login, user, password" lines before logging.
        std::string redact_cmd(const std::string &cmd) {
            if (cmd.rfind("login", 0) == 0) {
                return "login, <REDACTED>, <REDACTED>";
            }
            return cmd;
        }
    } // namespace

    bool is_valid_sbf_interval(const std::string &interval) {
        static const std::unordered_set<std::string> allowed{
            "onchange",
            "off",
            "msec5", "msec10", "msec20", "msec40", "msec50",
            "msec100", "msec200", "msec500",
            "sec1", "sec2", "sec5", "sec10", "sec15", "sec30", "sec60",
            "min2", "min5", "min10", "min15", "min30", "min60",
        };
        return allowed.find(lower_copy(interval)) != allowed.end();
    }

    std::string build_sbf_output_command(const SbfStream &stream, int ips_id) {
        if (stream.stream_id < 1 || stream.stream_id > 10) {
            throw ConfigError("SBF stream id must be in range 1..10");
        }
        if (ips_id < 1 || ips_id > 5) {
            throw ConfigError("IPS id must be in range 1..5");
        }
        if (stream.blocks.empty()) {
            throw ConfigError("SBF stream must contain at least one block");
        }
        if (!is_valid_sbf_interval(stream.interval)) {
            throw ConfigError("unsupported SBF interval '" + stream.interval + "'");
        }

        const std::string cmd =
            "setSBFOutput, Stream" + std::to_string(stream.stream_id) +
            ", IPS" + std::to_string(ips_id) +
            ", " + join(stream.blocks, '+') +
            ", " + stream.interval;
        enforce_command_length(cmd);
        return cmd;
    }

    ReceiverCapabilities parse_receiver_capabilities_reply(const std::string &reply) {
        const std::string payload = compact_payload_after_key(reply, "ReceiverCapabilities");
        const auto fields = split(payload, ',');
        if (fields.size() < 7) {
            throw ConfigError("ReceiverCapabilities reply had too few fields: " + reply);
        }

        ReceiverCapabilities caps;
        for (const auto &antenna: split(fields[0], '+')) {
            if (equals_ci(antenna, "Main")) caps.has_main = true;
            if (equals_ci(antenna, "Aux1")) caps.has_aux1 = true;
        }

        caps.measurement_interval_ms = parse_int_prefix(fields[fields.size() - 3], "measurement_interval_ms");
        caps.pvt_interval_ms = parse_int_prefix(fields[fields.size() - 2], "pvt_interval_ms");
        caps.ins_interval_ms = parse_int_prefix(fields[fields.size() - 1], "ins_interval_ms");
        return caps;
    }

    void verify_imu_orientation_reply(const std::string &reply, const ReceiverSettings &settings) {
        const auto fields = split(find_config_line_payload(reply, "IMUOrientation"), ',');
        if (fields.empty()) {
            throw ConfigError("IMUOrientation reply did not contain an orientation mode");
        }
        if (!equals_ci(fields[0], settings.imu_orientation_mode)) {
            throw ConfigError("IMU orientation mismatch: expected " +
                              settings.imu_orientation_mode + ", got " + fields[0]);
        }
        if (!equals_ci(settings.imu_orientation_mode, "SensorDefault")) {
            if (fields.size() < 4) {
                throw ConfigError("IMUOrientation reply did not include theta values");
            }
            const double theta_x = parse_double_prefix(fields[1], "ThetaX");
            const double theta_y = parse_double_prefix(fields[2], "ThetaY");
            const double theta_z = parse_double_prefix(fields[3], "ThetaZ");
            if (!close_enough(theta_x, settings.theta_x_deg) ||
                !close_enough(theta_y, settings.theta_y_deg) ||
                !close_enough(theta_z, settings.theta_z_deg)) {
                throw ConfigError("IMU orientation theta values do not match requested config");
            }
        }
    }

    void verify_ins_ant_lever_arm_reply(const std::string &reply, Vec3 expected) {
        const auto fields = split(find_config_line_payload(reply, "INSAntLeverArm"), ',');
        if (fields.size() < 3) {
            throw ConfigError("INSAntLeverArm reply did not contain x/y/z values");
        }
        const Vec3 actual{
            parse_double_prefix(fields[0], "INSAntLeverArm.X"),
            parse_double_prefix(fields[1], "INSAntLeverArm.Y"),
            parse_double_prefix(fields[2], "INSAntLeverArm.Z"),
        };
        if (!close_enough(actual.x, expected.x) ||
            !close_enough(actual.y, expected.y) ||
            !close_enough(actual.z, expected.z)) {
            throw ConfigError("INS antenna lever arm does not match requested config");
        }
    }

    void verify_gnss_attitude_reply(const std::string &reply, const std::string &expected_mode) {
        const auto fields = split(find_config_line_payload(reply, "GNSSAttitude"), ',');
        if (fields.empty()) {
            throw ConfigError("GNSSAttitude reply did not contain a mode");
        }
        if (!equals_ci(fields[0], expected_mode)) {
            throw ConfigError("GNSS attitude mode mismatch: expected " +
                              expected_mode + ", got " + fields[0]);
        }
    }

    void verify_attitude_offset_reply(const std::string &reply, AttitudeOffset expected) {
        const auto fields = split(find_config_line_payload(reply, "AttitudeOffset"), ',');
        if (fields.size() < 2) {
            throw ConfigError("AttitudeOffset reply did not contain heading/pitch values");
        }
        const double heading = parse_double_prefix(fields[0], "AttitudeOffset.Heading");
        const double pitch = parse_double_prefix(fields[1], "AttitudeOffset.Pitch");
        if (!close_enough(heading, expected.heading_deg) ||
            !close_enough(pitch, expected.pitch_deg)) {
            throw ConfigError("GNSS attitude offset does not match requested config");
        }
    }

    void validate_receiver_settings(const ReceiverSettings &settings) {
        if (settings.ips_id < 1 || settings.ips_id > 5) {
            throw ConfigError("receiver.ips_id must be in range 1..5");
        }
        if (!equals_ci(settings.imu_startup_data_mode, "Boot") &&
            !equals_ci(settings.imu_startup_data_mode, "GnssTimeKnown")) {
            throw ConfigError("receiver.imu.startup_data_mode must be Boot or GnssTimeKnown");
        }
        if (!equals_ci(settings.imu_orientation_mode, "SensorDefault") &&
            !equals_ci(settings.imu_orientation_mode, "manual") &&
            !equals_ci(settings.imu_orientation_mode, "fixed")) {
            throw ConfigError("receiver.imu.orientation_mode must be SensorDefault, manual, or fixed");
        }
        if (!settings.ant_lever_arm_configured) {
            throw ConfigError("receiver.imu.ant_lever_arm_m is required");
        }
        const auto in_lever_range = [](double v) { return v >= -100.0 && v <= 100.0; };
        if (!in_lever_range(settings.ant_lever_arm_m.x) ||
            !in_lever_range(settings.ant_lever_arm_m.y) ||
            !in_lever_range(settings.ant_lever_arm_m.z)) {
            throw ConfigError("receiver.imu.ant_lever_arm_m components must be in [-100, 100] meters");
        }
        if (!equals_ci(settings.gnss_attitude_mode, "none") &&
            !equals_ci(settings.gnss_attitude_mode, "MultiAntenna")) {
            throw ConfigError("receiver.gnss_attitude.mode must be none or MultiAntenna");
        }
        if (settings.streams.empty()) {
            throw ConfigError("receiver.streams must contain at least one stream");
        }
        for (const auto &s: settings.streams) {
            (void) build_sbf_output_command(s, settings.ips_id);
        }
    }

    std::string ReceiverConfigurator::send_command(const std::string &cmd, std::chrono::milliseconds timeout) {
        spdlog::debug("[receiver] -> {}", redact_cmd(cmd));
        ctrl_.write_all(cmd + "\r\n");

        // Receivers reply with a body and end with a "PROMPT>" line. Reading
        // until ">" is sufficient because all command echoes terminate with the
        // prompt.
        std::string reply = ctrl_.read_until(">", timeout);
        spdlog::debug("[receiver] <- {} bytes", reply.size());

        // "$R?" prefix or "USAGE:" anywhere in the reply indicates an error.
        if (reply.find("$R?") != std::string::npos || reply.find("Invalid") != std::string::npos) {
            throw ConfigError("receiver rejected '" + redact_cmd(cmd) + "': " + reply);
        }
        return reply;
    }

    void ReceiverConfigurator::login(const std::string &user, const std::string &password,
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

    void ReceiverConfigurator::apply(const ReceiverSettings &settings, std::chrono::milliseconds t) {
        validate_receiver_settings(settings);

        // 1) Check capabilities before configuring dual-antenna collection.
        {
            const auto caps = parse_receiver_capabilities_reply(send_command("getReceiverCapabilities", t));
            spdlog::info("[receiver] capabilities: main={} aux1={} meas={}ms pvt={}ms ins={}ms",
                         caps.has_main, caps.has_aux1,
                         caps.measurement_interval_ms, caps.pvt_interval_ms, caps.ins_interval_ms);
            if (settings.require_aux1 && !caps.has_aux1) {
                throw ConfigError("receiver capabilities do not include Aux1; dual-antenna collection is not available");
            }
        }

        // 2) Wipe all persistent SBF output streams so stale streams do not
        //    duplicate epochs or keep old rates alive.
        for (int stream_id = 1; stream_id <= 10; ++stream_id) {
            const std::string cmd =
                "setSBFOutput, Stream" + std::to_string(stream_id) + ", none, none, off";
            try { send_command(cmd, t); } catch (const ConfigError &e) {
                // Receiver may complain "Stream<id> not configured" — that's fine.
                spdlog::debug("[receiver] clear-stream{}: {}", stream_id, e.what());
            }
        }

        // 3) IPS data port (TCP server side on the receiver).
        {
            const std::string cmd =
                "setIPServerSettings, IPS" + std::to_string(settings.ips_id) +
                ", " + std::to_string(settings.ips_port) + ", TCP";
            send_command(cmd, t);
        }

        // 4) Enable SBF output on the IPS channel. Without this the receiver
        //    opens the TCP listener but never pushes any bytes. The empty middle
        //    field leaves the input direction unchanged.
        {
            const std::string cmd =
                "setDataInOut, IPS" + std::to_string(settings.ips_id) + ", , +SBF";
            send_command(cmd, t);
        }

        // 5) IMU startup mode, orientation, and antenna lever arm.
        send_command("setIMUStartupDataMode, " + settings.imu_startup_data_mode, t);
        send_command(set_orientation_command(settings), t);
        verify_imu_orientation_reply(send_command("getIMUOrientation", t), settings);

        send_command(set_vec3_command("setINSAntLeverArm", settings.ant_lever_arm_m), t);
        verify_ins_ant_lever_arm_reply(send_command("getINSAntLeverArm", t),
                                       settings.ant_lever_arm_m);

        // 6) Dual-antenna GNSS attitude setup and offset confirmation.
        {
            send_command("setGNSSAttitude, " + settings.gnss_attitude_mode, t);
            verify_gnss_attitude_reply(send_command("getGNSSAttitude", t),
                                       settings.gnss_attitude_mode);

            const std::string cmd =
                "setAttitudeOffset, " + fmt6(settings.attitude_offset_deg.heading_deg) +
                ", " + fmt6(settings.attitude_offset_deg.pitch_deg);
            send_command(cmd, t);
            verify_attitude_offset_reply(send_command("getAttitudeOffset", t),
                                         settings.attitude_offset_deg);
        }

        // 7) SBF stream membership + intervals.
        for (const auto &s: settings.streams) {
            if (s.blocks.empty()) continue;
            std::string blocks_csv = join(s.blocks, '+');
            send_command(build_sbf_output_command(s, settings.ips_id), t);
            spdlog::info("[receiver] Stream{} ({} @ {})",
                         s.stream_id, blocks_csv, s.interval);
        }
    }

    void ReceiverConfigurator::stop_streams(const ReceiverSettings &settings) noexcept {
        (void) settings;
        for (int stream_id = 1; stream_id <= 10; ++stream_id) {
            const std::string cmd =
                "setSBFOutput, Stream" + std::to_string(stream_id) + ", none, none, off";
            try { send_command(cmd, std::chrono::seconds(2)); } catch (...) {
                /* best-effort; the recorder is shutting down */
            }
        }
    }
} // namespace asterx
