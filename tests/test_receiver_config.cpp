// SPDX-License-Identifier: BSD-3-Clause
#include <algorithm>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "receiver_config.hpp"

namespace {

bool stream_contains(const std::vector<asterx::SbfStream>& streams,
                     const std::string& block) {
    for (const auto& stream: streams) {
        if (std::find(stream.blocks.begin(), stream.blocks.end(), block) != stream.blocks.end()) {
            return true;
        }
    }
    return false;
}

}  // namespace

TEST(ReceiverConfig, DefaultStreamsUseOnChangeAndRequiredBlocks) {
    asterx::ReceiverSettings settings;

    ASSERT_EQ(settings.streams.size(), 5u);
    for (const auto& stream: settings.streams) {
        EXPECT_EQ(stream.interval, "OnChange");
    }

    EXPECT_TRUE(stream_contains(settings.streams, "ExtSensorMeas"));
    EXPECT_TRUE(stream_contains(settings.streams, "MeasEpoch"));
    EXPECT_TRUE(stream_contains(settings.streams, "MeasExtra"));
    EXPECT_TRUE(stream_contains(settings.streams, "AuxAntPositions"));
    EXPECT_TRUE(stream_contains(settings.streams, "AttEuler"));
    EXPECT_TRUE(stream_contains(settings.streams, "IMUSetup"));
    EXPECT_TRUE(stream_contains(settings.streams, "ExtSensorStatus"));
    EXPECT_TRUE(stream_contains(settings.streams, "ReceiverStatus"));
    EXPECT_TRUE(stream_contains(settings.streams, "ChannelStatus"));
}

TEST(ReceiverConfig, ValidationRequiresLeverArm) {
    asterx::ReceiverSettings settings;
    EXPECT_THROW(asterx::validate_receiver_settings(settings), asterx::ConfigError);

    settings.ant_lever_arm_configured = true;
    settings.ant_lever_arm_m = asterx::Vec3{0.1, -0.2, 0.3};
    EXPECT_NO_THROW(asterx::validate_receiver_settings(settings));
}

TEST(ReceiverConfig, BuildsLongSbfOutputWithoutTruncation) {
    asterx::SbfStream stream;
    stream.stream_id = 6;
    stream.interval = "OnChange";
    for (int i = 0; i < 50; ++i) {
        stream.blocks.push_back("Block" + std::to_string(i));
    }

    const auto cmd = asterx::build_sbf_output_command(stream, 1);
    EXPECT_GT(cmd.size(), 256u);
    EXPECT_LT(cmd.size(), 2000u);
    EXPECT_NE(cmd.find("Block0+Block1"), std::string::npos);
    EXPECT_NE(cmd.find("Block49"), std::string::npos);
}

TEST(ReceiverConfig, BuildsLeverArmWithReceiverPrecision) {
    EXPECT_EQ(asterx::build_ins_ant_lever_arm_command(asterx::Vec3{0.0, 0.0, 0.0}),
              "setINSAntLeverArm, 0.000, 0.000, 0.000");
    EXPECT_EQ(asterx::build_ins_ant_lever_arm_command(asterx::Vec3{-0.0001, 1.2344, -2.3456}),
              "setINSAntLeverArm, 0.000, 1.234, -2.346");
}

TEST(ReceiverConfig, RejectsOversizedSbfOutputCommand) {
    asterx::SbfStream stream;
    stream.stream_id = 1;
    stream.interval = "OnChange";
    for (int i = 0; i < 250; ++i) {
        stream.blocks.push_back("VeryLongSyntheticBlockName" + std::to_string(i));
    }

    EXPECT_THROW((void) asterx::build_sbf_output_command(stream, 1),
                 asterx::ConfigError);
}

TEST(ReceiverConfig, ParsesReceiverCapabilities) {
    const std::string reply =
        "$R: grc\r\n"
        "  ReceiverCapabilities, Main+Aux1, GPSL1CA+GPSL5, COM1+IPS1,\r\n"
        "      APME+INS, 5, 100, 5\r\n"
        "COM1>";

    const auto caps = asterx::parse_receiver_capabilities_reply(reply);
    EXPECT_TRUE(caps.has_main);
    EXPECT_TRUE(caps.has_aux1);
    EXPECT_EQ(caps.measurement_interval_ms, 5);
    EXPECT_EQ(caps.pvt_interval_ms, 100);
    EXPECT_EQ(caps.ins_interval_ms, 5);
}

TEST(ReceiverConfig, VerifiesImuOrientationReply) {
    asterx::ReceiverSettings settings;
    settings.imu_orientation_mode = "manual";
    settings.theta_x_deg = -90.0;
    settings.theta_y_deg = 0.0;
    settings.theta_z_deg = 1.5;

    const std::string ok =
        "$R: gio\r\n"
        "  IMUOrientation, manual, -90.000, 0.000, 1.500\r\n"
        "COM1>";
    EXPECT_NO_THROW(asterx::verify_imu_orientation_reply(ok, settings));

    const std::string bad =
        "$R: gio\r\n"
        "  IMUOrientation, fixed, -90.000, 0.000, 1.500\r\n"
        "COM1>";
    EXPECT_THROW(asterx::verify_imu_orientation_reply(bad, settings),
                 asterx::ConfigError);
}

TEST(ReceiverConfig, VerifiesLeverArmAndAttitudeReplies) {
    EXPECT_NO_THROW(asterx::verify_ins_ant_lever_arm_reply(
        "$R: gial\r\n  INSAntLeverArm, 0.100, -0.200, 0.300\r\nCOM1>",
        asterx::Vec3{0.1, -0.2, 0.3}));

    EXPECT_NO_THROW(asterx::verify_gnss_attitude_reply(
        "$R: gga\r\n  GNSSAttitude, MultiAntenna\r\nCOM1>",
        "MultiAntenna"));

    EXPECT_NO_THROW(asterx::verify_attitude_offset_reply(
        "$R: gto\r\n  AttitudeOffset, 3.000, -1.500\r\nCOM1>",
        asterx::AttitudeOffset{3.0, -1.5}));
}
