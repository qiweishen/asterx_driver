#!/usr/bin/env python3
"""Decode Septentrio SBF recordings and export collected data.

Usage:
    python3 scripts/parser.py recordings/file.sbf
    python3 scripts/parser.py recordings/file.sbf --out-dir /tmp/export
    python3 scripts/parser.py recordings/file.sbf --summary-only

This parser is intentionally stdlib-only. It walks SBF frames directly,
validates the CRC, decodes the blocks used by the AsteRx collection driver,
exports GNSS measurements as per-antenna SBF files, and writes CSV files for
IMU, attitude, timing and receiver status data.
"""

from __future__ import annotations

import argparse
import csv
import math
import struct
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, Iterator, List, Optional


DEFAULT_PATH = "recordings/asterx-20260617T062847Z-1.sbf"

SBF_ID_NAMES = {
    4000: "MeasExtra",
    4002: "GALNav",
    4003: "GALAlm",
    4004: "GLONav",
    4005: "GLOAlm",
    4006: "PVTGeodetic",
    4007: "PVTCartesian",
    4013: "ChannelStatus",
    4014: "ReceiverStatus",
    4015: "Commands",
    4017: "GPSRawCA",
    4018: "GPSRawL2C",
    4019: "GPSRawL5",
    4020: "GEORawL1",
    4021: "GEORawL5",
    4022: "GALRawFNAV",
    4023: "GALRawINAV",
    4026: "GLORawCA",
    4027: "MeasEpoch",
    4030: "GALIon",
    4031: "GALUtc",
    4032: "GALGstGps",
    4034: "GALSARRLM",
    4036: "GLOTime",
    4042: "GPSCNav",
    4047: "BDSRaw",
    4049: "RTCMDatum",
    4066: "QZSRawL1CA",
    4067: "QZSRawL2C",
    4068: "QZSRawL5",
    4050: "ExtSensorMeas",
    4081: "BDSNav",
    4082: "QualityInd",
    4095: "QZSNav",
    4116: "QZSAlm",
    4119: "BDSAlm",
    4120: "BDSIon",
    4121: "BDSUtc",
    4245: "GALAuthStatus",
    4222: "ExtSensorInfo",
    4223: "ExtSensorStatus",
    4224: "IMUSetup",
    5891: "GPSNav",
    5892: "GPSAlm",
    5893: "GPSIon",
    5894: "GPSUtc",
    5896: "GEONav",
    5897: "GEOAlm",
    5902: "ReceiverSetup",
    5914: "ReceiverTime",
    5917: "GEOServiceLevel",
    5918: "GEONetworkTime",
    5919: "DiffCorrIn",
    5922: "EndOfMeas",
    5925: "GEOMT00",
    5926: "GEOPRNMask",
    5927: "GEOFastCorr",
    5928: "GEOIntegrity",
    5929: "GEOFastCorrDegr",
    5930: "GEODegrFactors",
    5931: "GEOIGPMask",
    5932: "GEOLongTermCorr",
    5933: "GEOIonoDelay",
    5934: "GEOClockEphCovMatrix",
    5938: "AttEuler",
    5939: "AttCovEuler",
    5942: "AuxAntPositions",
    5943: "EndOfAtt",
    5949: "BaseStation",
}

GNSS_COMMON_BLOCK_IDS = {
    4002, 4003, 4004, 4005,
    4015, 4017, 4018, 4019,
    4020, 4021, 4022, 4023, 4026,
    4030, 4031, 4032, 4034, 4036,
    4042, 4047, 4049, 4066, 4067, 4068,
    4081, 4095, 4116, 4119, 4120, 4121, 4245,
    5891, 5892, 5893, 5894, 5896, 5897, 5902,
    5917, 5918, 5919,
    5925, 5926, 5927, 5928, 5929, 5930, 5931, 5932, 5933, 5934,
    5949,
}

ANTENNA_NAMES = {
    0: "Main",
    1: "Aux1",
    2: "Aux2",
}

SIGNAL_TYPE_NAMES = {
    0: "GPS L1CA",
    1: "GPS L1P",
    2: "GPS L2P",
    3: "GPS L2C",
    4: "GPS L5",
    5: "GPS L1C",
    6: "QZS L1CA",
    7: "QZS L2C",
    8: "GLO L1CA",
    9: "GLO L1P",
    10: "GLO L2P",
    11: "GLO L2CA",
    12: "GLO L3",
    13: "BDS B1C",
    14: "BDS B2a",
    15: "NavIC L5",
    17: "GAL E1",
    19: "GAL E6",
    20: "GAL E5a",
    21: "GAL E5b",
    22: "GAL E5 AltBOC",
    23: "LBand",
    24: "SBAS L1CA",
    25: "SBAS L5",
    26: "QZS L5",
    27: "QZS L6",
    28: "BDS B1I",
    29: "BDS B2I",
    30: "BDS B3I",
    32: "QZS L1C",
    33: "QZS L1S",
    34: "BDS B2b",
    37: "NavIC L1",
    38: "QZS L1CB",
    39: "QZS L5S",
}

QUALITY_IND_NAMES = {
    0: "Overall quality",
    1: "GNSS signals from Main antenna",
    2: "GNSS signals from Aux1 antenna",
    11: "RF power level Main antenna",
    12: "RF power level Aux1 antenna",
    21: "CPU headroom",
    25: "OCXO stability",
    29: "Scintillation score",
    30: "Base-station measurements",
    31: "RTK post-processing prospect",
}

LIGHT_SPEED_MPS = 299_792_458.0
FLOAT_DNU = -2.0e10


@dataclass(frozen=True)
class SbfFrame:
    offset: int
    crc: int
    raw_id: int
    block_id: int
    revision: int
    length: int
    body: bytes


def crc_ccitt_xmodem(data: bytes) -> int:
    """CRC-16/XMODEM: poly=0x1021, init=0x0000, no reflection."""
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def rebuild_sbf_frame(raw_id: int, body: bytes) -> bytes:
    total = 8 + len(body)
    pad = (-total) % 4
    body_padded = body + b"\x00" * pad
    length = 8 + len(body_padded)
    if length > 8188:
        raise ValueError(f"SBF frame too large: {length} bytes")
    crc_input = struct.pack("<HH", raw_id, length) + body_padded
    crc = crc_ccitt_xmodem(crc_input)
    return struct.pack("<BBHHH", 0x24, 0x40, crc, raw_id, length) + body_padded


def iter_frames(data: bytes) -> Iterator[SbfFrame]:
    """Yield CRC-valid SBF frames from a byte buffer."""
    i = 0
    n = len(data)
    while i + 8 <= n:
        if data[i] != 0x24 or data[i + 1] != 0x40:
            i += 1
            continue

        crc, raw_id, length = struct.unpack_from("<HHH", data, i + 2)
        if length < 8 or length > 8188 or length % 4 != 0 or i + length > n:
            i += 1
            continue

        frame_bytes = data[i + 4 : i + length]
        if crc_ccitt_xmodem(frame_bytes) != crc:
            i += 1
            continue

        yield SbfFrame(
            offset=i,
            crc=crc,
            raw_id=raw_id,
            block_id=raw_id & 0x1FFF,
            revision=(raw_id >> 13) & 0x07,
            length=length,
            body=data[i + 8 : i + length],
        )
        i += length


def raw_scan(path: Path) -> Dict[str, Any]:
    data = path.read_bytes()
    counts: Counter[int] = Counter()
    revisions: Dict[int, int] = {}
    crc_failed_by_id: Counter[int] = Counter()
    crc_fails = 0
    recognized_bytes = 0
    frames = 0

    i = 0
    n = len(data)
    while i + 8 <= n:
        if data[i] != 0x24 or data[i + 1] != 0x40:
            i += 1
            continue

        crc, raw_id, length = struct.unpack_from("<HHH", data, i + 2)
        block_id = raw_id & 0x1FFF
        revision = (raw_id >> 13) & 0x07
        if length < 8 or length > 8188 or length % 4 != 0 or i + length > n:
            i += 1
            continue

        if crc_ccitt_xmodem(data[i + 4 : i + length]) != crc:
            crc_fails += 1
            crc_failed_by_id[block_id] += 1
            i += 1
            continue

        counts[block_id] += 1
        revisions[block_id] = max(revisions.get(block_id, revision), revision)
        recognized_bytes += length
        frames += 1
        i += length

    return {
        "file_size": len(data),
        "head_hex": data[:32].hex(),
        "frames": frames,
        "recognized_bytes": recognized_bytes,
        "id_counts": counts,
        "rev_counts": revisions,
        "crc_failed_by_id": crc_failed_by_id,
        "crc_fails": crc_fails,
    }


def dnu_float(value: float) -> Optional[float]:
    if not math.isfinite(value) or value <= FLOAT_DNU * 0.5:
        return None
    return value


def dnu_u(value: int, dnu: int) -> Optional[int]:
    return None if value == dnu else value


def dnu_i(value: int, dnu: int) -> Optional[int]:
    return None if value == dnu else value


def sign_extend(value: int, bits: int) -> int:
    sign_bit = 1 << (bits - 1)
    return (value ^ sign_bit) - sign_bit


def signal_type(type_byte: int, aux_bits: int) -> int:
    sig_lo = type_byte & 0x1F
    if sig_lo == 31:
        return 32 + ((aux_bits >> 3) & 0x1F)
    return sig_lo


def antenna_id(type_byte: int) -> int:
    return (type_byte >> 5) & 0x07


def antenna_name(antenna: int) -> str:
    return ANTENNA_NAMES.get(antenna, f"Ant{antenna}")


def signal_name(signal: int) -> str:
    return SIGNAL_TYPE_NAMES.get(signal, f"sig{signal}")


def glonass_freq_number(signal: int, info_bits: int) -> Optional[int]:
    if signal not in (8, 9, 10, 11, 12):
        return None
    return ((info_bits >> 3) & 0x1F) - 8


def carrier_frequency_hz(
    signal: int, info_bits: int = 0, glo_k: Optional[int] = None
) -> Optional[float]:
    """Best-effort carrier frequency by Septentrio signal type."""
    if glo_k is None:
        glo_k = glonass_freq_number(signal, info_bits)
    if signal in (0, 1, 5, 6, 17, 24, 28, 32, 33, 37, 38):
        return 1575.42e6
    if signal in (2, 3, 7):
        return 1227.60e6
    if signal in (4, 15, 20, 25, 26, 39):
        return 1176.45e6
    if signal in (8, 9) and glo_k is not None:
        return 1602.0e6 + glo_k * 0.5625e6
    if signal in (10, 11) and glo_k is not None:
        return 1246.0e6 + glo_k * 0.4375e6
    if signal == 12 and glo_k is not None:
        return 1202.025e6 + glo_k * 0.4375e6
    if signal == 13:
        return 1575.42e6
    if signal == 14:
        return 1176.45e6
    if signal == 19:
        return 1278.75e6
    if signal == 21:
        return 1207.14e6
    if signal == 22:
        return 1191.795e6
    if signal == 27:
        return 1278.75e6
    if signal == 29:
        return 1207.14e6
    if signal == 30:
        return 1268.52e6
    if signal == 34:
        return 1207.14e6
    return None


def cn0_dbhz(cn0_raw: int, signal: int) -> Optional[float]:
    if cn0_raw == 255:
        return None
    if signal in (1, 2):
        return cn0_raw * 0.25
    return cn0_raw * 0.25 + 10.0


def printable_ascii(data: bytes) -> str:
    cleaned = data.split(b"\x00", 1)[0]
    return "".join(chr(b) if 32 <= b < 127 else "." for b in cleaned)


def gps_time_fields(body: bytes) -> Dict[str, Any]:
    row: Dict[str, Any] = {}
    if len(body) >= 4:
        row["tow_ms"] = struct.unpack_from("<I", body, 0)[0]
    if len(body) >= 6:
        row["wnc"] = struct.unpack_from("<H", body, 4)[0]
    return row


class CsvSink:
    def __init__(self, out_dir: Path):
        self.out_dir = out_dir
        self.files: Dict[str, Any] = {}
        self.writers: Dict[str, csv.DictWriter[str]] = {}
        self.headers: Dict[str, List[str]] = {}
        out_dir.mkdir(parents=True, exist_ok=True)

    def writer(self, filename: str, headers: List[str]) -> csv.DictWriter[str]:
        if filename in self.writers:
            return self.writers[filename]
        path = self.out_dir / filename
        handle = path.open("w", newline="")
        writer = csv.DictWriter(handle, fieldnames=headers, extrasaction="ignore")
        writer.writeheader()
        self.files[filename] = handle
        self.writers[filename] = writer
        self.headers[filename] = headers
        return writer

    def row(self, filename: str, headers: List[str], row: Dict[str, Any]) -> None:
        clean = {key: format_csv_value(row.get(key)) for key in headers}
        self.writer(filename, headers).writerow(clean)

    def close(self) -> None:
        for handle in self.files.values():
            handle.close()


def format_csv_value(value: Any) -> Any:
    if value is None:
        return ""
    if isinstance(value, float):
        if not math.isfinite(value):
            return ""
        return f"{value:.12g}"
    return value


IMU_HEADERS = [
    "GPS_Week",
    "GPS_MS[ms]",
    "Acc_X[m/s^2]",
    "Acc_Y[m/s^2]",
    "Acc_Z[m/s^2]",
    "Gyro_X[deg/s]",
    "Gyro_Y[deg/s]",
    "Gyro_Z[deg/s]",
]

IMU_SAMPLE_HEADERS = [
    "tow_ms",
    "wnc",
    "source",
    "sensor_model",
    "meas_type",
    "meas_type_name",
    "x",
    "y",
    "z",
]

ATTITUDE_HEADERS = [
    "tow_ms",
    "wnc",
    "nr_sv",
    "error",
    "mode",
    "heading_deg",
    "pitch_deg",
    "roll_deg",
    "pitch_rate_deg_s",
    "roll_rate_deg_s",
    "heading_rate_deg_s",
]

ATT_COV_HEADERS = [
    "tow_ms",
    "wnc",
    "error",
    "cov_head_head_deg2",
    "cov_pitch_pitch_deg2",
    "cov_roll_roll_deg2",
    "cov_head_pitch_deg2",
    "cov_head_roll_deg2",
    "cov_pitch_roll_deg2",
]

AUX_ANT_HEADERS = [
    "tow_ms",
    "wnc",
    "aux_index",
    "aux_ant_id",
    "nr_sv",
    "error",
    "ambiguity_type",
    "delta_east_m",
    "delta_north_m",
    "delta_up_m",
    "east_vel_m_s",
    "north_vel_m_s",
    "up_vel_m_s",
]

IMU_SETUP_HEADERS = [
    "tow_ms",
    "wnc",
    "reserved",
    "serial_port",
    "ant_lever_arm_x_m",
    "ant_lever_arm_y_m",
    "ant_lever_arm_z_m",
    "theta_x_deg",
    "theta_y_deg",
    "theta_z_deg",
    "estimated_theta_x_deg",
    "estimated_theta_y_deg",
    "estimated_theta_z_deg",
    "constraint_lever_arm_x_m",
    "constraint_lever_arm_y_m",
    "constraint_lever_arm_z_m",
]

QUALITY_HEADERS = [
    "tow_ms",
    "wnc",
    "indicator_type",
    "indicator_name",
    "value",
]

RECEIVER_TIME_HEADERS = [
    "tow_ms",
    "wnc",
    "utc_year",
    "utc_month",
    "utc_day",
    "utc_hour",
    "utc_min",
    "utc_sec",
    "delta_ls",
    "sync_level",
]

RECEIVER_STATUS_HEADERS = [
    "tow_ms",
    "wnc",
    "cpu_load_pct",
    "ext_error",
    "uptime_s",
    "rx_state",
    "rx_error",
    "agc_index",
    "front_end_id",
    "front_end_code",
    "antenna_id",
    "antenna",
    "gain_db",
    "sample_var",
    "blanking_stat_pct",
    "cmd_count",
    "temperature_c",
]

CHANNEL_STATUS_HEADERS = [
    "tow_ms",
    "wnc",
    "sat_index",
    "state_index",
    "svid",
    "svid_full",
    "freq_nr_raw",
    "azimuth_deg",
    "rise_set",
    "health_status",
    "elevation_deg",
    "rx_channel",
    "antenna_id",
    "antenna",
    "tracking_status",
    "pvt_status",
    "pvt_info",
]

EXT_SENSOR_STATUS_HEADERS = [
    "tow_ms",
    "wnc",
    "source",
    "sensor_model",
    "status_flags",
    "info_type",
    "info_value",
    "data_hex",
]

EXT_SENSOR_INFO_HEADERS = [
    "tow_ms",
    "wnc",
    "source",
    "sensor_model",
    "product_code",
    "firmware_revision",
    "serial_number",
    "data_ascii",
    "data_hex",
]

RAW_BLOCK_HEADERS = [
    "tow_ms",
    "wnc",
    "block_id",
    "block_name",
    "revision",
    "length",
    "frame_offset",
    "body_hex",
]

BLOCK_SUMMARY_HEADERS = [
    "block_id",
    "block_name",
    "revision",
    "count",
]


def parse_type1_measurement(
    body: bytes,
    off: int,
    epoch_index: int,
    obs_index: int,
    frame: SbfFrame,
    common_flags: int,
    cum_clk_jumps: int,
    tow: int,
    wnc: int,
) -> Dict[str, Any]:
    rx_ch = body[off]
    type_b = body[off + 1]
    svid = body[off + 2]
    misc = body[off + 3]
    code_lsb = struct.unpack_from("<I", body, off + 4)[0]
    doppler_raw = struct.unpack_from("<i", body, off + 8)[0]
    carrier_lsb = struct.unpack_from("<H", body, off + 12)[0]
    carrier_msb = struct.unpack_from("<b", body, off + 14)[0]
    cn0_raw = body[off + 15]
    lock = struct.unpack_from("<H", body, off + 16)[0]
    obs_info = body[off + 18]

    sig = signal_type(type_b, obs_info)
    ant = antenna_id(type_b)
    glo_k = glonass_freq_number(sig, obs_info)
    code_msb = misc & 0x0F
    pseudorange = None
    if code_msb != 0 or code_lsb != 0:
        pseudorange = (code_msb * 4_294_967_296 + code_lsb) * 0.001

    doppler = None if doppler_raw == -2_147_483_648 else doppler_raw * 0.0001
    freq = carrier_frequency_hz(sig, obs_info, glo_k)
    carrier_phase = None
    if pseudorange is not None and not (carrier_msb == -128 and carrier_lsb == 0):
        if freq is not None:
            wavelength = LIGHT_SPEED_MPS / freq
            carrier_phase = (
                pseudorange / wavelength
                + (carrier_msb * 65_536 + carrier_lsb) * 0.001
            )

    return {
        "tow_ms": tow,
        "wnc": wnc,
        "epoch_index": epoch_index,
        "obs_index": obs_index,
        "level": 1,
        "parent_obs_index": "",
        "rx_channel": rx_ch,
        "antenna_id": ant,
        "antenna": antenna_name(ant),
        "svid": svid,
        "signal_type": sig,
        "signal_name": signal_name(sig),
        "glonass_freq_number": glo_k,
        "pseudorange_m": pseudorange,
        "carrier_phase_cycles": carrier_phase,
        "doppler_hz": doppler,
        "cn0_dbhz": cn0_dbhz(cn0_raw, sig),
        "cn0_raw": None if cn0_raw == 255 else cn0_raw,
        "lock_time_s": dnu_u(lock, 65535),
        "smoothed": bool(obs_info & 0x01),
        "half_cycle": bool(obs_info & 0x04),
        "raw_type": type_b,
        "obs_info": obs_info,
        "common_flags": common_flags,
        "cum_clk_jumps_ms": cum_clk_jumps,
        "frame_offset": frame.offset,
        "_freq_hz": freq,
        "_glo_freq_number": glo_k,
    }


def parse_type2_measurement(
    body: bytes,
    off: int,
    parent: Dict[str, Any],
    epoch_index: int,
    obs_index: int,
    frame: SbfFrame,
    common_flags: int,
    cum_clk_jumps: int,
    tow: int,
    wnc: int,
) -> Dict[str, Any]:
    type_b = body[off]
    lock = body[off + 1]
    cn0_raw = body[off + 2]
    offsets_msb = body[off + 3]
    carrier_msb = struct.unpack_from("<b", body, off + 4)[0]
    obs_info = body[off + 5]
    code_offset_lsb = struct.unpack_from("<H", body, off + 6)[0]
    carrier_lsb = struct.unpack_from("<H", body, off + 8)[0]
    doppler_offset_lsb = struct.unpack_from("<H", body, off + 10)[0]

    sig = signal_type(type_b, obs_info)
    ant = antenna_id(type_b)
    parent_glo = parent.get("_glo_freq_number")
    if sig in (8, 9, 10, 11, 12):
        glo_k = parent_glo
    else:
        glo_k = glonass_freq_number(sig, obs_info)
    code_offset_msb = sign_extend(offsets_msb & 0x07, 3)
    doppler_offset_msb = sign_extend((offsets_msb >> 3) & 0x1F, 5)

    parent_pr = parent.get("pseudorange_m")
    pseudorange = None
    if parent_pr is not None and not (
        code_offset_msb == -4 and code_offset_lsb == 0
    ):
        pseudorange = parent_pr + (
            code_offset_msb * 65_536 + code_offset_lsb
        ) * 0.001

    parent_doppler = parent.get("doppler_hz")
    parent_freq = parent.get("_freq_hz")
    freq = carrier_frequency_hz(sig, obs_info, glo_k)
    doppler = None
    if parent_doppler is not None and not (
        doppler_offset_msb == -16 and doppler_offset_lsb == 0
    ):
        alpha = (freq / parent_freq) if freq is not None and parent_freq else 1.0
        doppler = (
            parent_doppler * alpha
            + (doppler_offset_msb * 65_536 + doppler_offset_lsb) * 0.0001
        )

    carrier_phase = None
    if pseudorange is not None and not (carrier_msb == -128 and carrier_lsb == 0):
        if freq is not None:
            wavelength = LIGHT_SPEED_MPS / freq
            carrier_phase = (
                pseudorange / wavelength
                + (carrier_msb * 65_536 + carrier_lsb) * 0.001
            )

    return {
        "tow_ms": tow,
        "wnc": wnc,
        "epoch_index": epoch_index,
        "obs_index": obs_index,
        "level": 2,
        "parent_obs_index": parent["obs_index"],
        "rx_channel": parent["rx_channel"],
        "antenna_id": ant,
        "antenna": antenna_name(ant),
        "svid": parent["svid"],
        "signal_type": sig,
        "signal_name": signal_name(sig),
        "glonass_freq_number": glo_k,
        "pseudorange_m": pseudorange,
        "carrier_phase_cycles": carrier_phase,
        "doppler_hz": doppler,
        "cn0_dbhz": cn0_dbhz(cn0_raw, sig),
        "cn0_raw": None if cn0_raw == 255 else cn0_raw,
        "lock_time_s": dnu_u(lock, 255),
        "smoothed": bool(obs_info & 0x01),
        "half_cycle": bool(obs_info & 0x04),
        "raw_type": type_b,
        "obs_info": obs_info,
        "common_flags": common_flags,
        "cum_clk_jumps_ms": cum_clk_jumps,
        "frame_offset": frame.offset,
        "_freq_hz": freq,
        "_glo_freq_number": glo_k,
    }


def decode_meas_epoch(frame: SbfFrame) -> List[Dict[str, Any]]:
    body = frame.body
    if len(body) < 12:
        return []

    tow = struct.unpack_from("<I", body, 0)[0]
    wnc = struct.unpack_from("<H", body, 4)[0]
    n1 = body[6]
    sb1_len = body[7]
    sb2_len = body[8]
    common_flags = body[9]
    cum_clk_jumps = body[10]

    rows: List[Dict[str, Any]] = []
    off = 12
    obs_index = 0
    for epoch_index in range(n1):
        if sb1_len < 20 or off + sb1_len > len(body):
            break

        parent = parse_type1_measurement(
            body,
            off,
            epoch_index,
            obs_index,
            frame,
            common_flags,
            cum_clk_jumps,
            tow,
            wnc,
        )
        rows.append(parent)
        obs_index += 1
        n2 = body[off + 19]

        type2_off = off + sb1_len
        for _ in range(n2):
            if sb2_len < 12 or type2_off + sb2_len > len(body):
                break
            child = parse_type2_measurement(
                body,
                type2_off,
                parent,
                epoch_index,
                obs_index,
                frame,
                common_flags,
                cum_clk_jumps,
                tow,
                wnc,
            )
            rows.append(child)
            obs_index += 1
            type2_off += sb2_len

        off += sb1_len + n2 * sb2_len

    for row in rows:
        row.pop("_freq_hz", None)
        row.pop("_glo_freq_number", None)
    return rows


def decode_meas_extra(frame: SbfFrame) -> List[Dict[str, Any]]:
    body = frame.body
    if len(body) < 12:
        return []

    tow = struct.unpack_from("<I", body, 0)[0]
    wnc = struct.unpack_from("<H", body, 4)[0]
    n_mod = body[6]
    sb_len = body[7]
    doppler_var_factor = struct.unpack_from("<f", body, 8)[0]
    if sb_len < 16:
        return []

    payload_len = max(0, len(body) - 12)
    nr_subblocks = ((payload_len // sb_len - n_mod) // 256) * 256 + n_mod
    if nr_subblocks <= 0 or nr_subblocks * sb_len > payload_len:
        nr_subblocks = payload_len // sb_len

    rows = []
    off = 12
    for idx in range(nr_subblocks):
        if off + sb_len > len(body):
            break
        rx_ch = body[off]
        type_b = body[off + 1]
        mp_corr = struct.unpack_from("<h", body, off + 2)[0] * 0.001
        smooth_corr = struct.unpack_from("<h", body, off + 4)[0] * 0.001
        code_var_raw = struct.unpack_from("<H", body, off + 6)[0]
        carrier_var_raw = struct.unpack_from("<H", body, off + 8)[0]
        lock = struct.unpack_from("<H", body, off + 10)[0]
        cum_loss = body[off + 12]
        carrier_mp_corr = struct.unpack_from("<b", body, off + 13)[0]
        info = body[off + 14]
        misc = body[off + 15]
        sig = signal_type(type_b, misc)
        ant = antenna_id(type_b)
        doppler_var = None
        if carrier_var_raw != 65535:
            doppler_var = carrier_var_raw * doppler_var_factor
        rows.append(
            {
                "tow_ms": tow,
                "wnc": wnc,
                "extra_index": idx,
                "rx_channel": rx_ch,
                "antenna_id": ant,
                "antenna": antenna_name(ant),
                "signal_type": sig,
                "signal_name": signal_name(sig),
                "glonass_freq_number": glonass_freq_number(sig, misc),
                "mp_correction_m": mp_corr,
                "smoothing_correction_m": smooth_corr,
                "code_var_m2": None
                if code_var_raw == 65535
                else code_var_raw * 0.0001,
                "carrier_var_mcycle2": None
                if carrier_var_raw == 65535
                else carrier_var_raw,
                "doppler_var_factor": doppler_var_factor,
                "doppler_var_mhz2": doppler_var,
                "lock_time_s": dnu_u(lock, 65535),
                "cum_loss_cont": cum_loss,
                "carrier_mp_corr_cycles": carrier_mp_corr / 512.0,
                "cn0_highres_extension_dbhz": (misc & 0x07) * 0.03125,
                "info": info,
                "misc": misc,
                "frame_offset": frame.offset,
            }
        )
        off += sb_len
    return rows


def decode_ext_sensor_meas(frame: SbfFrame) -> List[Dict[str, Any]]:
    body = frame.body
    if len(body) < 8:
        return []
    tow = struct.unpack_from("<I", body, 0)[0]
    wnc = struct.unpack_from("<H", body, 4)[0]
    n = body[6]
    sb_len = body[7]
    if sb_len < 28:
        return []

    rows = []
    off = 8
    type_names = {
        0: "acceleration_m_s2",
        1: "angular_rate_deg_s",
    }
    for _ in range(n):
        if off + sb_len > len(body):
            break
        source = body[off]
        sensor_model = body[off + 1]
        meas_type = body[off + 2]
        x, y, z = struct.unpack_from("<ddd", body, off + 4)
        rows.append(
            {
                "tow_ms": tow,
                "wnc": wnc,
                "source": source,
                "sensor_model": sensor_model,
                "meas_type": meas_type,
                "meas_type_name": type_names.get(meas_type, f"type_{meas_type}"),
                "x": x,
                "y": y,
                "z": z,
            }
        )
        off += sb_len
    return rows


def decode_ext_sensor_meas_epoch(frame: SbfFrame) -> Optional[Dict[str, Any]]:
    body = frame.body
    if len(body) < 8:
        return None
    tow = struct.unpack_from("<I", body, 0)[0]
    wnc = struct.unpack_from("<H", body, 4)[0]
    n = body[6]
    sb_len = body[7]
    if sb_len < 28:
        return None

    row: Dict[str, Any] = {
        "GPS_Week": wnc,
        "GPS_MS[ms]": tow,
        "Acc_X[m/s^2]": None,
        "Acc_Y[m/s^2]": None,
        "Acc_Z[m/s^2]": None,
        "Gyro_X[deg/s]": None,
        "Gyro_Y[deg/s]": None,
        "Gyro_Z[deg/s]": None,
    }
    off = 8
    for _ in range(n):
        if off + sb_len > len(body):
            break
        meas_type = body[off + 2]
        if meas_type in (0, 1):
            x, y, z = struct.unpack_from("<ddd", body, off + 4)
            if meas_type == 0:
                row["Acc_X[m/s^2]"] = x
                row["Acc_Y[m/s^2]"] = y
                row["Acc_Z[m/s^2]"] = z
            else:
                row["Gyro_X[deg/s]"] = x
                row["Gyro_Y[deg/s]"] = y
                row["Gyro_Z[deg/s]"] = z
        off += sb_len
    return row


def decode_att_euler(frame: SbfFrame) -> Optional[Dict[str, Any]]:
    body = frame.body
    if len(body) < 36:
        return None
    return {
        "tow_ms": struct.unpack_from("<I", body, 0)[0],
        "wnc": struct.unpack_from("<H", body, 4)[0],
        "nr_sv": dnu_u(body[6], 255),
        "error": body[7],
        "mode": struct.unpack_from("<H", body, 8)[0],
        "heading_deg": dnu_float(struct.unpack_from("<f", body, 12)[0]),
        "pitch_deg": dnu_float(struct.unpack_from("<f", body, 16)[0]),
        "roll_deg": dnu_float(struct.unpack_from("<f", body, 20)[0]),
        "pitch_rate_deg_s": dnu_float(struct.unpack_from("<f", body, 24)[0]),
        "roll_rate_deg_s": dnu_float(struct.unpack_from("<f", body, 28)[0]),
        "heading_rate_deg_s": dnu_float(struct.unpack_from("<f", body, 32)[0]),
    }


def decode_att_cov_euler(frame: SbfFrame) -> Optional[Dict[str, Any]]:
    body = frame.body
    if len(body) < 32:
        return None
    return {
        "tow_ms": struct.unpack_from("<I", body, 0)[0],
        "wnc": struct.unpack_from("<H", body, 4)[0],
        "error": body[7],
        "cov_head_head_deg2": dnu_float(struct.unpack_from("<f", body, 8)[0]),
        "cov_pitch_pitch_deg2": dnu_float(struct.unpack_from("<f", body, 12)[0]),
        "cov_roll_roll_deg2": dnu_float(struct.unpack_from("<f", body, 16)[0]),
        "cov_head_pitch_deg2": dnu_float(struct.unpack_from("<f", body, 20)[0]),
        "cov_head_roll_deg2": dnu_float(struct.unpack_from("<f", body, 24)[0]),
        "cov_pitch_roll_deg2": dnu_float(struct.unpack_from("<f", body, 28)[0]),
    }


def decode_aux_ant_positions(frame: SbfFrame) -> List[Dict[str, Any]]:
    body = frame.body
    if len(body) < 8:
        return []
    tow = struct.unpack_from("<I", body, 0)[0]
    wnc = struct.unpack_from("<H", body, 4)[0]
    n = body[6]
    sb_len = body[7]
    if sb_len < 52:
        return []

    rows = []
    off = 8
    for idx in range(n):
        if off + sb_len > len(body):
            break
        rows.append(
            {
                "tow_ms": tow,
                "wnc": wnc,
                "aux_index": idx,
                "aux_ant_id": body[off + 3],
                "nr_sv": dnu_u(body[off], 255),
                "error": body[off + 1],
                "ambiguity_type": dnu_u(body[off + 2], 255),
                "delta_east_m": dnu_float(struct.unpack_from("<d", body, off + 4)[0]),
                "delta_north_m": dnu_float(struct.unpack_from("<d", body, off + 12)[0]),
                "delta_up_m": dnu_float(struct.unpack_from("<d", body, off + 20)[0]),
                "east_vel_m_s": dnu_float(struct.unpack_from("<d", body, off + 28)[0]),
                "north_vel_m_s": dnu_float(struct.unpack_from("<d", body, off + 36)[0]),
                "up_vel_m_s": dnu_float(struct.unpack_from("<d", body, off + 44)[0]),
            }
        )
        off += sb_len
    return rows


def decode_imu_setup(frame: SbfFrame) -> Optional[Dict[str, Any]]:
    body = frame.body
    if len(body) < 8:
        return None
    row: Dict[str, Any] = {
        "tow_ms": struct.unpack_from("<I", body, 0)[0],
        "wnc": struct.unpack_from("<H", body, 4)[0],
        "reserved": body[6],
        "serial_port": body[7],
    }
    names = [
        "ant_lever_arm_x_m",
        "ant_lever_arm_y_m",
        "ant_lever_arm_z_m",
        "theta_x_deg",
        "theta_y_deg",
        "theta_z_deg",
        "estimated_theta_x_deg",
        "estimated_theta_y_deg",
        "estimated_theta_z_deg",
        "constraint_lever_arm_x_m",
        "constraint_lever_arm_y_m",
        "constraint_lever_arm_z_m",
    ]
    off = 8
    for name in names:
        row[name] = None
        if off + 4 <= len(body):
            row[name] = dnu_float(struct.unpack_from("<f", body, off)[0])
        off += 4
    return row


def decode_quality_ind(frame: SbfFrame) -> List[Dict[str, Any]]:
    body = frame.body
    if len(body) < 8:
        return []
    tow = struct.unpack_from("<I", body, 0)[0]
    wnc = struct.unpack_from("<H", body, 4)[0]
    n = body[6]
    rows = []
    for idx in range(n):
        off = 8 + idx * 2
        if off + 2 > len(body):
            break
        packed = struct.unpack_from("<H", body, off)[0]
        ind_type = packed & 0xFF
        value = (packed >> 8) & 0x0F
        rows.append(
            {
                "tow_ms": tow,
                "wnc": wnc,
                "indicator_type": ind_type,
                "indicator_name": QUALITY_IND_NAMES.get(ind_type, f"type_{ind_type}"),
                "value": None if value == 15 else value,
            }
        )
    return rows


def decode_receiver_time(frame: SbfFrame) -> Optional[Dict[str, Any]]:
    body = frame.body
    if len(body) < 14:
        return None
    utc_year, utc_month, utc_day, utc_hour, utc_min, utc_sec, delta_ls = (
        struct.unpack_from("<bbbbbbb", body, 6)
    )
    return {
        "tow_ms": struct.unpack_from("<I", body, 0)[0],
        "wnc": struct.unpack_from("<H", body, 4)[0],
        "utc_year": dnu_i(utc_year, -128),
        "utc_month": dnu_i(utc_month, -128),
        "utc_day": dnu_i(utc_day, -128),
        "utc_hour": dnu_i(utc_hour, -128),
        "utc_min": dnu_i(utc_min, -128),
        "utc_sec": dnu_i(utc_sec, -128),
        "delta_ls": dnu_i(delta_ls, -128),
        "sync_level": body[13],
    }


def decode_receiver_status(frame: SbfFrame) -> List[Dict[str, Any]]:
    body = frame.body
    if len(body) < 24:
        return []
    tow = struct.unpack_from("<I", body, 0)[0]
    wnc = struct.unpack_from("<H", body, 4)[0]
    cpu = body[6]
    ext_error = body[7]
    uptime = struct.unpack_from("<I", body, 8)[0]
    rx_state = struct.unpack_from("<I", body, 12)[0]
    rx_error = struct.unpack_from("<I", body, 16)[0]
    n = body[20]
    sb_len = body[21]
    cmd_count = body[22]
    temp_raw = body[23]
    temp_c = None if temp_raw == 0 else temp_raw - 100
    rows = []
    off = 24
    for idx in range(max(n, 1)):
        base = {
            "tow_ms": tow,
            "wnc": wnc,
            "cpu_load_pct": dnu_u(cpu, 255),
            "ext_error": ext_error,
            "uptime_s": uptime,
            "rx_state": rx_state,
            "rx_error": rx_error,
            "agc_index": "" if n == 0 else idx,
            "cmd_count": dnu_u(cmd_count, 0),
            "temperature_c": temp_c,
        }
        if n == 0:
            rows.append(base)
            break
        if sb_len < 4 or off + sb_len > len(body):
            break
        front_end_id = body[off]
        gain = struct.unpack_from("<b", body, off + 1)[0]
        ant = (front_end_id >> 5) & 0x07
        base.update(
            {
                "front_end_id": front_end_id,
                "front_end_code": front_end_id & 0x1F,
                "antenna_id": ant,
                "antenna": antenna_name(ant),
                "gain_db": dnu_i(gain, -128),
                "sample_var": dnu_u(body[off + 2], 0),
                "blanking_stat_pct": body[off + 3],
            }
        )
        rows.append(base)
        off += sb_len
    return rows


def decode_channel_status(frame: SbfFrame) -> List[Dict[str, Any]]:
    body = frame.body
    if len(body) < 12:
        return []
    tow = struct.unpack_from("<I", body, 0)[0]
    wnc = struct.unpack_from("<H", body, 4)[0]
    n = body[6]
    sb1_len = body[7]
    sb2_len = body[8]
    if sb1_len < 12 or sb2_len < 8:
        return []

    rows = []
    off = 12
    for sat_idx in range(n):
        if off + sb1_len > len(body):
            break
        svid = body[off]
        freq_nr = body[off + 1]
        svid_full = struct.unpack_from("<H", body, off + 2)[0]
        az_rs = struct.unpack_from("<H", body, off + 4)[0]
        health = struct.unpack_from("<H", body, off + 6)[0]
        elevation = struct.unpack_from("<b", body, off + 8)[0]
        n2 = body[off + 9]
        rx_channel = body[off + 10]
        azimuth = az_rs & 0x01FF
        if azimuth == 511:
            azimuth = None
        state_off = off + sb1_len
        for state_idx in range(n2):
            if state_off + sb2_len > len(body):
                break
            ant = body[state_off]
            rows.append(
                {
                    "tow_ms": tow,
                    "wnc": wnc,
                    "sat_index": sat_idx,
                    "state_index": state_idx,
                    "svid": svid,
                    "svid_full": svid_full,
                    "freq_nr_raw": freq_nr,
                    "azimuth_deg": azimuth,
                    "rise_set": (az_rs >> 14) & 0x03,
                    "health_status": health,
                    "elevation_deg": dnu_i(elevation, -128),
                    "rx_channel": rx_channel,
                    "antenna_id": ant,
                    "antenna": antenna_name(ant),
                    "tracking_status": struct.unpack_from("<H", body, state_off + 2)[0],
                    "pvt_status": struct.unpack_from("<H", body, state_off + 4)[0],
                    "pvt_info": struct.unpack_from("<H", body, state_off + 6)[0],
                }
            )
            state_off += sb2_len
        off += sb1_len + n2 * sb2_len
    return rows


def decode_ext_sensor_status(frame: SbfFrame) -> Optional[Dict[str, Any]]:
    body = frame.body
    if len(body) < 8:
        return None
    row = {
        "tow_ms": struct.unpack_from("<I", body, 0)[0],
        "wnc": struct.unpack_from("<H", body, 4)[0],
        "source": body[6],
        "sensor_model": body[7],
        "status_flags": "",
        "info_type": "",
        "info_value": "",
        "data_hex": body[8:].hex(),
    }
    if body[7] == 10 and len(body) >= 12:
        row["status_flags"] = body[8]
        row["info_type"] = body[9]
        row["info_value"] = struct.unpack_from("<H", body, 10)[0]
    return row


def decode_ext_sensor_info(frame: SbfFrame) -> Optional[Dict[str, Any]]:
    body = frame.body
    if len(body) < 8:
        return None
    data = body[8:]
    row = {
        "tow_ms": struct.unpack_from("<I", body, 0)[0],
        "wnc": struct.unpack_from("<H", body, 4)[0],
        "source": body[6],
        "sensor_model": body[7],
        "product_code": "",
        "firmware_revision": "",
        "serial_number": "",
        "data_ascii": printable_ascii(data),
        "data_hex": data.hex(),
    }
    if body[7] == 10 and len(data) >= 28:
        row["product_code"] = printable_ascii(data[:10])
        row["firmware_revision"] = printable_ascii(data[10:26])
        row["serial_number"] = struct.unpack_from("<H", data, 26)[0]
    return row


def raw_block_row(frame: SbfFrame) -> Dict[str, Any]:
    row = gps_time_fields(frame.body)
    row.update(
        {
            "block_id": frame.block_id,
            "block_name": SBF_ID_NAMES.get(frame.block_id, "Unknown"),
            "revision": frame.revision,
            "length": frame.length,
            "frame_offset": frame.offset,
            "body_hex": frame.body.hex(),
        }
    )
    return row


class SbfSink:
    def __init__(self, out_dir: Path):
        self.out_dir = out_dir
        self.files: Dict[int, Any] = {}
        self.paths: Dict[int, Path] = {}
        self.frames: Counter[int] = Counter()
        self.bytes: Counter[int] = Counter()
        self.common_history: List[bytes] = []
        self.common_frames = 0
        out_dir.mkdir(parents=True, exist_ok=True)

    def path_for(self, ant: int) -> Path:
        return self.out_dir / f"gnss-ant{ant}-{antenna_name(ant)}.sbf"

    def write(self, ant: int, frame_bytes: bytes) -> None:
        if ant not in self.files:
            path = self.path_for(ant)
            self.paths[ant] = path
            self.files[ant] = path.open("wb")
            for common_frame in self.common_history:
                self._write_open(ant, common_frame)
        self._write_open(ant, frame_bytes)

    def write_common(self, frame_bytes: bytes) -> None:
        self.common_history.append(frame_bytes)
        self.common_frames += 1
        for ant in list(self.files):
            self._write_open(ant, frame_bytes)

    def _write_open(self, ant: int, frame_bytes: bytes) -> None:
        self.files[ant].write(frame_bytes)
        self.frames[ant] += 1
        self.bytes[ant] += len(frame_bytes)

    def close(self) -> None:
        for handle in self.files.values():
            handle.close()


def cleanup_generated_gnss_outputs(out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    for path in out_dir.glob("gnss-ant*.sbf"):
        path.unlink()
    for stale_name in ("gnss_observations.csv", "gnss_extras.csv"):
        stale_path = out_dir / stale_name
        if stale_path.exists():
            stale_path.unlink()


def type_byte_for_observation(row: Dict[str, Any]) -> int:
    signal = int(row["signal_type"])
    sig_lo = 31 if signal >= 32 else signal & 0x1F
    return ((int(row["antenna_id"]) & 0x07) << 5) | sig_lo


def obs_info_for_type1(row: Dict[str, Any]) -> int:
    signal = int(row["signal_type"])
    info = 0
    if row.get("smoothed"):
        info |= 0x01
    if row.get("half_cycle"):
        info |= 0x04
    if signal >= 32:
        info |= ((signal - 32) & 0x1F) << 3
    elif signal in (8, 9, 10, 11):
        glo_k = row.get("glonass_freq_number")
        if glo_k is not None and glo_k != "":
            info |= ((int(glo_k) + 8) & 0x1F) << 3
    return info & 0xFF


def encode_code_fields(pseudorange_m: Optional[float]) -> tuple[int, int]:
    if pseudorange_m is None:
        return 0, 0
    code_units = int(round(pseudorange_m * 1000.0))
    if code_units <= 0:
        return 0, 0
    return (code_units >> 32) & 0x0F, code_units & 0xFFFFFFFF


def encode_doppler_raw(doppler_hz: Optional[float]) -> int:
    if doppler_hz is None:
        return -2_147_483_648
    raw = int(round(doppler_hz * 10_000.0))
    return max(-2_147_483_647, min(2_147_483_647, raw))


def encode_carrier_fields(row: Dict[str, Any], obs_info: int) -> tuple[int, int]:
    pseudorange = row.get("pseudorange_m")
    carrier_phase = row.get("carrier_phase_cycles")
    if pseudorange is None or carrier_phase is None:
        return 0, -128

    signal = int(row["signal_type"])
    glo_k = row.get("glonass_freq_number")
    if glo_k == "":
        glo_k = None
    freq = carrier_frequency_hz(
        signal, obs_info, None if glo_k is None else int(glo_k)
    )
    if freq is None:
        return 0, -128

    wavelength = LIGHT_SPEED_MPS / freq
    relative_cycles = carrier_phase - pseudorange / wavelength
    relative_mcycles = int(round(relative_cycles * 1000.0))
    carrier_msb = relative_mcycles // 65_536
    carrier_lsb = relative_mcycles - carrier_msb * 65_536
    if carrier_msb < -127 or carrier_msb > 127:
        return 0, -128
    return carrier_lsb & 0xFFFF, carrier_msb


def encode_type1_observation(row: Dict[str, Any]) -> bytes:
    obs_info = obs_info_for_type1(row)
    code_msb, code_lsb = encode_code_fields(row.get("pseudorange_m"))
    carrier_lsb, carrier_msb = encode_carrier_fields(row, obs_info)
    cn0_raw = row.get("cn0_raw")
    if cn0_raw is None or cn0_raw == "":
        cn0_raw = 255
    lock_time = row.get("lock_time_s")
    if lock_time is None or lock_time == "":
        lock_time = 65535
    lock_time = max(0, min(65535, int(lock_time)))
    return struct.pack(
        "<BBBBIiHbBHBB",
        int(row["rx_channel"]) & 0xFF,
        type_byte_for_observation(row),
        int(row["svid"]) & 0xFF,
        code_msb & 0x0F,
        code_lsb,
        encode_doppler_raw(row.get("doppler_hz")),
        carrier_lsb,
        carrier_msb,
        int(cn0_raw) & 0xFF,
        lock_time,
        obs_info,
        0,
    )


def rows_by_antenna(rows: List[Dict[str, Any]]) -> Dict[int, List[Dict[str, Any]]]:
    grouped: Dict[int, List[Dict[str, Any]]] = defaultdict(list)
    for row in rows:
        grouped[int(row["antenna_id"])].append(row)
    return grouped


def build_meas_epoch_body(rows: List[Dict[str, Any]]) -> bytes:
    if not rows:
        raise ValueError("cannot build MeasEpoch without observations")
    first = rows[0]
    if len(rows) > 255:
        raise ValueError(f"too many observations for one MeasEpoch: {len(rows)}")
    header = struct.pack(
        "<IHBBBBBB",
        int(first["tow_ms"]),
        int(first["wnc"]),
        len(rows),
        20,
        12,
        int(first["common_flags"]) & 0xFF,
        int(first["cum_clk_jumps_ms"]) & 0xFF,
        0,
    )
    return header + b"".join(encode_type1_observation(row) for row in rows)


def meas_extra_subblocks(frame: SbfFrame) -> tuple[bytes, int, List[bytes]]:
    body = frame.body
    if len(body) < 12:
        return b"", 0, []
    n_mod = body[6]
    sb_len = body[7]
    if sb_len == 0:
        return b"", 0, []
    payload_len = max(0, len(body) - 12)
    nr_subblocks = ((payload_len // sb_len - n_mod) // 256) * 256 + n_mod
    if nr_subblocks <= 0 or nr_subblocks * sb_len > payload_len:
        nr_subblocks = payload_len // sb_len
    blocks = []
    off = 12
    for _ in range(nr_subblocks):
        if off + sb_len > len(body):
            break
        blocks.append(bytes(body[off : off + sb_len]))
        off += sb_len
    return bytes(body[:12]), sb_len, blocks


def build_meas_extra_body(frame: SbfFrame, indices: List[int]) -> Optional[bytes]:
    header, sb_len, blocks = meas_extra_subblocks(frame)
    if not header or not sb_len or not indices:
        return None
    kept = bytearray()
    for idx in indices:
        if idx < len(blocks):
            kept.extend(blocks[idx])
    if not kept:
        return None
    return bytes(header[:6]) + bytes([len(kept) // sb_len % 256, sb_len]) + header[8:12] + bytes(kept)


def verify_sbf_bytes(data: bytes) -> tuple[int, int]:
    frames = 0
    crc_fails = 0
    i = 0
    n = len(data)
    while i + 8 <= n:
        if data[i] != 0x24 or data[i + 1] != 0x40:
            i += 1
            continue
        crc, _, length = struct.unpack_from("<HHH", data, i + 2)
        if length < 8 or length > 8188 or length % 4 != 0 or i + length > n:
            i += 1
            continue
        if crc_ccitt_xmodem(data[i + 4 : i + length]) != crc:
            crc_fails += 1
            i += 1
            continue
        frames += 1
        i += length
    return frames, crc_fails


def export_recording(path: Path, out_dir: Path) -> Dict[str, Any]:
    data = path.read_bytes()
    frames = list(iter_frames(data))
    cleanup_generated_gnss_outputs(out_dir)
    sink = CsvSink(out_dir)
    sbf_sink = SbfSink(out_dir)
    block_counts: Counter[int] = Counter()
    revisions: Dict[int, int] = {}
    gnss_type1_by_ant: Counter[int] = Counter()
    gnss_all_by_ant: Counter[int] = Counter()
    extras_by_ant: Counter[int] = Counter()
    pending_extra_indices: Dict[int, List[int]] = {}
    pending_epoch_antennas: set[int] = set()
    pending_epoch_time: Optional[tuple[int, int]] = None
    emitted: Counter[str] = Counter()

    try:
        for frame in frames:
            block_counts[frame.block_id] += 1
            revisions[frame.block_id] = max(
                revisions.get(frame.block_id, frame.revision), frame.revision
            )

            if frame.block_id == 4027:
                rows = decode_meas_epoch(frame)
                grouped = rows_by_antenna(rows)
                pending_extra_indices = defaultdict(list)
                pending_epoch_antennas = set(grouped)
                pending_epoch_time = None

                if rows:
                    pending_epoch_time = (
                        int(rows[0]["tow_ms"]),
                        int(rows[0]["wnc"]),
                    )

                for idx, row in enumerate(rows):
                    ant = int(row["antenna_id"])
                    pending_extra_indices[ant].append(idx)
                    gnss_all_by_ant[ant] += 1
                    if int(row["level"]) == 1:
                        gnss_type1_by_ant[ant] += 1

                for ant, ant_rows in sorted(grouped.items()):
                    body = build_meas_epoch_body(ant_rows)
                    sbf_sink.write(ant, rebuild_sbf_frame(frame.raw_id, body))
                continue

            if frame.block_id == 4000:
                extra_time = None
                if len(frame.body) >= 6:
                    extra_time = (
                        struct.unpack_from("<I", frame.body, 0)[0],
                        struct.unpack_from("<H", frame.body, 4)[0],
                    )
                if pending_extra_indices and extra_time == pending_epoch_time:
                    for ant, indices in sorted(pending_extra_indices.items()):
                        body = build_meas_extra_body(frame, indices)
                        if body:
                            sbf_sink.write(ant, rebuild_sbf_frame(frame.raw_id, body))
                            extras_by_ant[ant] += len(indices)
                continue

            if frame.block_id == 5922:
                for ant in sorted(pending_epoch_antennas):
                    sbf_sink.write(ant, rebuild_sbf_frame(frame.raw_id, frame.body))
                pending_extra_indices = {}
                pending_epoch_antennas = set()
                pending_epoch_time = None
                continue

            if frame.block_id in GNSS_COMMON_BLOCK_IDS:
                sbf_sink.write_common(rebuild_sbf_frame(frame.raw_id, frame.body))
                continue

            if frame.block_id == 4050:
                epoch_row = decode_ext_sensor_meas_epoch(frame)
                if epoch_row:
                    sink.row("imu.csv", IMU_HEADERS, epoch_row)
                    emitted["imu"] += 1
                for row in decode_ext_sensor_meas(frame):
                    sink.row("imu_samples.csv", IMU_SAMPLE_HEADERS, row)
                    emitted["imu_samples"] += 1
                continue

            if frame.block_id == 5938:
                row = decode_att_euler(frame)
                if row:
                    sink.row("attitude.csv", ATTITUDE_HEADERS, row)
                    emitted["attitude"] += 1
                continue

            if frame.block_id == 5939:
                row = decode_att_cov_euler(frame)
                if row:
                    sink.row("attitude_cov.csv", ATT_COV_HEADERS, row)
                    emitted["attitude_cov"] += 1
                continue

            if frame.block_id == 5942:
                for row in decode_aux_ant_positions(frame):
                    sink.row("aux_ant_positions.csv", AUX_ANT_HEADERS, row)
                    emitted["aux_ant_positions"] += 1
                continue

            if frame.block_id == 4224:
                row = decode_imu_setup(frame)
                if row:
                    sink.row("imu_setup.csv", IMU_SETUP_HEADERS, row)
                    emitted["imu_setup"] += 1
                continue

            if frame.block_id == 4082:
                for row in decode_quality_ind(frame):
                    sink.row("quality_ind.csv", QUALITY_HEADERS, row)
                    emitted["quality_ind"] += 1
                continue

            if frame.block_id == 5914:
                row = decode_receiver_time(frame)
                if row:
                    sink.row("receiver_time.csv", RECEIVER_TIME_HEADERS, row)
                    emitted["receiver_time"] += 1
                continue

            if frame.block_id == 4014:
                for row in decode_receiver_status(frame):
                    sink.row("receiver_status.csv", RECEIVER_STATUS_HEADERS, row)
                    emitted["receiver_status"] += 1
                continue

            if frame.block_id == 4013:
                for row in decode_channel_status(frame):
                    sink.row("channel_status.csv", CHANNEL_STATUS_HEADERS, row)
                    emitted["channel_status"] += 1
                continue

            if frame.block_id == 4223:
                row = decode_ext_sensor_status(frame)
                if row:
                    sink.row(
                        "ext_sensor_status.csv", EXT_SENSOR_STATUS_HEADERS, row
                    )
                    emitted["ext_sensor_status"] += 1
                continue

            if frame.block_id == 4222:
                row = decode_ext_sensor_info(frame)
                if row:
                    sink.row("ext_sensor_info.csv", EXT_SENSOR_INFO_HEADERS, row)
                    emitted["ext_sensor_info"] += 1
                continue

            if frame.block_id not in (5922, 5943):
                sink.row("raw_blocks.csv", RAW_BLOCK_HEADERS, raw_block_row(frame))
                emitted["raw_blocks"] += 1

        for block_id, count in sorted(block_counts.items()):
            sink.row(
                "block_summary.csv",
                BLOCK_SUMMARY_HEADERS,
                {
                    "block_id": block_id,
                    "block_name": SBF_ID_NAMES.get(block_id, "Unknown"),
                    "revision": revisions.get(block_id, ""),
                    "count": count,
                },
            )

    finally:
        sink.close()
        sbf_sink.close()

    gnss_sbf_verify: Dict[int, tuple[int, int]] = {}
    for ant, sbf_path in sbf_sink.paths.items():
        gnss_sbf_verify[ant] = verify_sbf_bytes(sbf_path.read_bytes())

    return {
        "frames": len(frames),
        "block_counts": block_counts,
        "revisions": revisions,
        "gnss_type1_by_ant": gnss_type1_by_ant,
        "gnss_all_by_ant": gnss_all_by_ant,
        "extras_by_ant": extras_by_ant,
        "gnss_sbf_paths": sbf_sink.paths,
        "gnss_sbf_frames_by_ant": sbf_sink.frames,
        "gnss_sbf_bytes_by_ant": sbf_sink.bytes,
        "gnss_sbf_verify": gnss_sbf_verify,
        "gnss_common_frames": sbf_sink.common_frames,
        "emitted": emitted,
        "out_dir": out_dir,
    }


def print_raw_summary(path: Path) -> None:
    result = raw_scan(path)
    print("\n=== Raw-byte scan ===")
    print(f"File: {path} ({result['file_size']:,} bytes)")
    print(f"First 32 bytes: {result['head_hex']}")
    print(
        f"Frames (CRC-valid): {result['frames']:,}   "
        f"Recognized bytes: {result['recognized_bytes']:,}"
    )
    print(
        f"CRC failures: {result['crc_fails']:,}   "
        f"Unrecognized bytes: "
        f"{result['file_size'] - result['recognized_bytes']:,}"
    )
    print()
    print(f"  {'BlockID':>8}  {'Rev':>3}  {'Name':<22}  {'OK count':>10}")
    all_ids = sorted(result["id_counts"], key=lambda bid: -result["id_counts"][bid])
    for block_id in all_ids:
        print(
            f"  {block_id:>8}  "
            f"{result['rev_counts'].get(block_id, 0):>3}  "
            f"{SBF_ID_NAMES.get(block_id, '(unmapped ID)'):<22}  "
            f"{result['id_counts'][block_id]:>10}"
        )
    if result["crc_failed_by_id"]:
        print("\nCRC failures by block ID:")
        for block_id, count in result["crc_failed_by_id"].most_common():
            print(f"  {block_id:>8}  {count:>10}")


def print_export_summary(summary: Dict[str, Any]) -> None:
    print("\n=== Export summary ===")
    print(f"Output directory: {summary['out_dir']}")
    print(f"CRC-valid frames exported/scanned: {summary['frames']:,}")
    print()
    print("GNSS SBF files written:")
    if not summary["gnss_sbf_paths"]:
        print("  (no GNSS measurement SBF files)")
    for ant, path in sorted(summary["gnss_sbf_paths"].items()):
        frames = summary["gnss_sbf_frames_by_ant"].get(ant, 0)
        byte_count = summary["gnss_sbf_bytes_by_ant"].get(ant, 0)
        verified_frames, crc_fails = summary["gnss_sbf_verify"].get(ant, (0, 0))
        print(
            f"  {antenna_name(ant):<8} {path}  "
            f"{frames:,} frames, {byte_count:,} bytes, "
            f"verify={verified_frames:,} frames/{crc_fails} crc_fail"
        )
    print(f"  common nav/context frames copied: {summary['gnss_common_frames']:,}")
    if summary["gnss_common_frames"] == 0:
        print("  warning: no GNSS nav/context blocks were present in this recording")

    print("\nCSV rows written:")
    for name, count in sorted(summary["emitted"].items()):
        print(f"  {name:<24} {count:>10}")

    print("\nMeasEpoch observations by antenna:")
    ants = sorted(
        set(summary["gnss_type1_by_ant"]) | set(summary["gnss_all_by_ant"])
    )
    if not ants:
        print("  (no MeasEpoch observations)")
    for ant in ants:
        type1 = summary["gnss_type1_by_ant"].get(ant, 0)
        all_obs = summary["gnss_all_by_ant"].get(ant, 0)
        print(
            f"  {antenna_name(ant):<8} "
            f"Type1={type1:>8}  all_signals={all_obs:>8}"
        )

    if summary["extras_by_ant"]:
        print("\nMeasExtra SBF sub-blocks by antenna:")
        for ant, count in sorted(summary["extras_by_ant"].items()):
            print(f"  {antenna_name(ant):<8} {count:>10}")


def parse_args(argv: Iterable[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "path",
        nargs="?",
        default=DEFAULT_PATH,
        help=f"SBF file to parse (default: {DEFAULT_PATH})",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        help="output directory (default: <input-stem>-parsed next to input)",
    )
    parser.add_argument(
        "--summary-only",
        action="store_true",
        help="only print the raw SBF block summary; do not write CSV files",
    )
    return parser.parse_args(list(argv))


def main(argv: Iterable[str]) -> int:
    args = parse_args(argv)
    path = Path(args.path)
    if not path.exists():
        print(f"error: input file does not exist: {path}", file=sys.stderr)
        return 2

    print_raw_summary(path)
    if args.summary_only:
        return 0

    out_dir = args.out_dir
    if out_dir is None:
        out_dir = path.with_name(f"{path.stem}-parsed")
    summary = export_recording(path, out_dir)
    print_export_summary(summary)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
