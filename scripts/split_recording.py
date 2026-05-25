#!/usr/bin/env python3
"""Split a recorded .sbf into per-antenna SBF files + an IMU CSV.

Usage:
    python scripts/split_recording.py <recording.sbf> [output_dir]

Outputs (in output_dir, default = the input file's directory):
    <basename>-main.sbf   all blocks except ExtSensorMeas; MeasEpoch / MeasExtra
                          filtered to AntennaID = 0 sub-blocks; CRCs recomputed.
    <basename>-aux1.sbf   same, AntennaID = 1.
    <basename>-imu.csv    ExtSensorMeas decoded into one row per epoch:
                            GPS_Week,GPS_MS[ms],
                            Acc_X[m/s^2],Acc_Y[m/s^2],Acc_Z[m/s^2],
                            Gyro_X[deg/s],Gyro_Y[deg/s],Gyro_Z[deg/s]

Both .sbf outputs are valid SBF streams ready to feed into sbf2rin / convbin.
"""
import csv
import math
import os
import struct
import sys
from collections import defaultdict


# ---------------------------------------------------------------------------
# CRC and frame rebuild
# ---------------------------------------------------------------------------

def crc_ccitt_xmodem(data):
    """CRC-16/XMODEM: poly=0x1021, init=0x0000, no reflection."""
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def rebuild_sbf_frame(raw_id, body):
    """Pack a complete on-wire SBF frame from (raw_id, body).

    Pads the body so the total frame length is a multiple of 4, computes the
    CRC-CCITT-XMODEM over (ID + Length + padded body), and returns the bytes
    ready to write.
    """
    body_size = len(body)
    total     = 8 + body_size
    pad       = (-total) % 4
    body_pad  = body + b"\x00" * pad
    length    = 8 + len(body_pad)

    crc_input = struct.pack("<HH", raw_id, length) + body_pad
    crc       = crc_ccitt_xmodem(crc_input)

    header = struct.pack("<BBHHH", 0x24, 0x40, crc, raw_id, length)
    return header + body_pad


# ---------------------------------------------------------------------------
# MeasEpoch (4027) split by AntennaID
# ---------------------------------------------------------------------------

def split_meas_epoch(body):
    """Split a MeasEpoch v2 body. Returns:
       per_ant_body: dict {ant: bytes} — new body bytes per antenna (no SBF header)
       keep_idx:     dict {ant: [type1 indices kept]} — used to align MeasExtra
       aux2_dropped: int — number of Type1 sub-blocks with AntennaID >= 2 we dropped
    """
    if len(body) < 12:
        return {}, {}, 0

    tow         = struct.unpack_from("<I", body, 0)[0]
    wnc         = struct.unpack_from("<H", body, 4)[0]
    n1          = body[6]
    sb1l        = body[7]
    sb2l        = body[8]
    common_tail = bytes(body[9:12])    # CommonFlags, CumClkJumps, Reserved

    keep_groups = defaultdict(bytearray)   # ant -> raw bytes of kept Type1+Type2 groups
    keep_idx    = defaultdict(list)        # ant -> list of Type1 indices kept
    aux2_dropped = 0

    off = 12
    for idx in range(n1):
        if off + sb1l > len(body):
            break
        n2 = body[off + 19]
        group_size = sb1l + n2 * sb2l
        if off + group_size > len(body):
            break
        group     = body[off : off + group_size]
        # Per the AsteRx firmware reference, the Type byte (sub-block offset 1)
        # packs SignalType in bits 0-4 and AntennaID in bits 5-7. ObsInfo
        # (offset 18) carries lock / smoothing flags, NOT AntennaID.
        type_byte = body[off + 1]
        ant       = (type_byte >> 5) & 0x07

        if ant in (0, 1):
            keep_groups[ant].extend(group)
            keep_idx[ant].append(idx)
        else:
            aux2_dropped += 1

        off += group_size

    per_ant_body = {}
    for ant, groups in keep_groups.items():
        new_n1 = len(keep_idx[ant])
        header = (struct.pack("<I", tow)
                  + struct.pack("<H", wnc)
                  + bytes([new_n1, sb1l, sb2l])
                  + common_tail)
        per_ant_body[ant] = bytes(header) + bytes(groups)

    return per_ant_body, keep_idx, aux2_dropped


# ---------------------------------------------------------------------------
# MeasExtra (4000) split by AntennaID, using the index map from MeasEpoch.
#
# MeasExtra contains N sub-blocks aligned 1:1 with MeasEpoch's Type1 sub-blocks
# (same TOW, same order). Drop the same indices the MeasEpoch split dropped.
# ---------------------------------------------------------------------------

def split_meas_extra(body, keep_idx):
    if len(body) < 12:
        return {}

    tow         = struct.unpack_from("<I", body, 0)[0]
    wnc         = struct.unpack_from("<H", body, 4)[0]
    n           = body[6]
    sbl         = body[7]
    header_tail = bytes(body[8:12])    # DopplerVarFactor + reserved bytes

    # Pull out each sub-block's raw bytes by index.
    sub_off = 12
    sub_bytes = []
    for _ in range(n):
        if sub_off + sbl > len(body):
            break
        sub_bytes.append(bytes(body[sub_off : sub_off + sbl]))
        sub_off += sbl

    per_ant_body = {}
    for ant, indices in keep_idx.items():
        kept = bytearray()
        actual_n = 0
        for idx in indices:
            if idx < len(sub_bytes):
                kept.extend(sub_bytes[idx])
                actual_n += 1
        header = (struct.pack("<I", tow)
                  + struct.pack("<H", wnc)
                  + bytes([actual_n, sbl])
                  + header_tail)
        per_ant_body[ant] = bytes(header) + bytes(kept)

    return per_ant_body


# ---------------------------------------------------------------------------
# ExtSensorMeas (4050) decoder — pulls accel + gyro from a single epoch.
#
# Each Type1 sub-block (typical SBLength = 28 bytes):
#   byte 0: Source        (32 = internal SPI for the AsteRx-i3 built-in IMU)
#   byte 1: SensorModel
#   byte 2: Type          (0 = Accel,  1 = AngularRate,  3 = Info, ...)
#   byte 3: ObsInfo / reserved
#   bytes 4..11:  X (f8)
#   bytes 12..19: Y (f8)
#   bytes 20..27: Z (f8)
#
# Accel is m/s^2 (SBF). Gyro is rad/s (SBF) — we convert to deg/s for the CSV.
# ---------------------------------------------------------------------------

def decode_ext_sensor_meas(body):
    if len(body) < 8:
        return None
    tow = struct.unpack_from("<I", body, 0)[0]
    wnc = struct.unpack_from("<H", body, 4)[0]
    n   = body[6]
    sbl = body[7]

    ax = ay = az = None
    wx = wy = wz = None
    off = 8
    for _ in range(n):
        if off + sbl > len(body):
            break
        typ = body[off + 2]
        if sbl >= 28 and typ in (0, 1):
            x = struct.unpack_from("<d", body, off + 4)[0]
            y = struct.unpack_from("<d", body, off + 12)[0]
            z = struct.unpack_from("<d", body, off + 20)[0]
            if typ == 0:
                ax, ay, az = x, y, z
            else:
                wx, wy, wz = math.degrees(x), math.degrees(y), math.degrees(z)
        off += sbl

    return tow, wnc, ax, ay, az, wx, wy, wz


# ---------------------------------------------------------------------------
# Output verification: re-scan an emitted .sbf and validate every CRC.
# ---------------------------------------------------------------------------

def verify_sbf(path):
    with open(path, "rb") as f:
        data = f.read()
    frames = 0
    crc_fails = 0
    i, n = 0, len(data)
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


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

def fmt(n):
    return f"{n:,}"


def main(argv):
    if len(argv) < 2:
        print("Usage: split_recording.py <input.sbf> [output_dir]", file=sys.stderr)
        return 2

    in_path = argv[1]
    out_dir = argv[2] if len(argv) >= 3 else (os.path.dirname(in_path) or ".")
    os.makedirs(out_dir, exist_ok=True)

    base      = os.path.splitext(os.path.basename(in_path))[0]
    main_path = os.path.join(out_dir, f"{base}-main.sbf")
    aux1_path = os.path.join(out_dir, f"{base}-aux1.sbf")
    imu_path  = os.path.join(out_dir, f"{base}-imu.csv")

    with open(in_path, "rb") as f:
        data = f.read()
    n = len(data)

    stats = {
        "frames_total":         0,
        "meas_epoch":           0,
        "meas_extra":           0,
        "ext_sensor_meas":      0,
        "passthrough":          0,
        "main_subblocks":       0,
        "aux1_subblocks":       0,
        "aux2_subblocks":       0,
        "imu_rows":             0,
        "crc_failures":         0,
        "extra_no_meas":        0,
    }

    last_keep_idx = {}    # MeasEpoch -> MeasExtra correspondence (same TOW)
    last_meas_tow = None

    with open(main_path, "wb") as f_main, \
         open(aux1_path, "wb") as f_aux1, \
         open(imu_path,  "w", newline="") as f_csv:

        csv_w = csv.writer(f_csv)
        csv_w.writerow([
            "GPS_Week", "GPS_MS[ms]",
            "Acc_X[m/s^2]", "Acc_Y[m/s^2]", "Acc_Z[m/s^2]",
            "Gyro_X[deg/s]", "Gyro_Y[deg/s]", "Gyro_Z[deg/s]",
        ])

        i = 0
        while i + 8 <= n:
            if data[i] != 0x24 or data[i + 1] != 0x40:
                i += 1
                continue
            crc, raw_id, length = struct.unpack_from("<HHH", data, i + 2)
            bid = raw_id & 0x1FFF
            if length < 8 or length > 8188 or length % 4 != 0 or i + length > n:
                i += 1
                continue
            if crc_ccitt_xmodem(data[i + 4 : i + length]) != crc:
                stats["crc_failures"] += 1
                i += 1
                continue

            stats["frames_total"] += 1
            body = data[i + 8 : i + length]

            if bid == 4027:                                # MeasEpoch
                stats["meas_epoch"] += 1
                per_ant, keep_idx, aux2 = split_meas_epoch(body)
                stats["aux2_subblocks"] += aux2
                last_keep_idx = keep_idx
                last_meas_tow = struct.unpack_from("<I", body, 0)[0]
                stats["main_subblocks"] += len(keep_idx.get(0, []))
                stats["aux1_subblocks"] += len(keep_idx.get(1, []))
                if 0 in per_ant:
                    f_main.write(rebuild_sbf_frame(raw_id, per_ant[0]))
                if 1 in per_ant:
                    f_aux1.write(rebuild_sbf_frame(raw_id, per_ant[1]))

            elif bid == 4000:                              # MeasExtra
                stats["meas_extra"] += 1
                this_tow = struct.unpack_from("<I", body, 0)[0]
                if not last_keep_idx or this_tow != last_meas_tow:
                    stats["extra_no_meas"] += 1
                else:
                    per_ant = split_meas_extra(body, last_keep_idx)
                    if 0 in per_ant:
                        f_main.write(rebuild_sbf_frame(raw_id, per_ant[0]))
                    if 1 in per_ant:
                        f_aux1.write(rebuild_sbf_frame(raw_id, per_ant[1]))

            elif bid == 4050:                              # ExtSensorMeas
                stats["ext_sensor_meas"] += 1
                dec = decode_ext_sensor_meas(body)
                if dec is not None:
                    tow, wnc, ax, ay, az, wx, wy, wz = dec
                    csv_w.writerow([
                        wnc, tow,
                        "" if ax is None else f"{ax:.10g}",
                        "" if ay is None else f"{ay:.10g}",
                        "" if az is None else f"{az:.10g}",
                        "" if wx is None else f"{wx:.10g}",
                        "" if wy is None else f"{wy:.10g}",
                        "" if wz is None else f"{wz:.10g}",
                    ])
                    stats["imu_rows"] += 1

            else:                                          # pass-through
                stats["passthrough"] += 1
                frame_bytes = data[i : i + length]
                f_main.write(frame_bytes)
                f_aux1.write(frame_bytes)

            i += length

    # ------------------------------------------------------------------
    # Verify output files: every emitted frame must validate CRC.
    # ------------------------------------------------------------------
    main_frames, main_bad = verify_sbf(main_path)
    aux1_frames, aux1_bad = verify_sbf(aux1_path)

    # ------------------------------------------------------------------
    # Print summary
    # ------------------------------------------------------------------
    print(f"Input:  {in_path}  ({fmt(n)} bytes)")
    print(f"Outputs:")
    print(f"  {main_path}  ({fmt(os.path.getsize(main_path))} bytes, "
          f"{fmt(main_frames)} frames, {main_bad} CRC fails)")
    print(f"  {aux1_path}  ({fmt(os.path.getsize(aux1_path))} bytes, "
          f"{fmt(aux1_frames)} frames, {aux1_bad} CRC fails)")
    print(f"  {imu_path}   ({fmt(stats['imu_rows'])} rows)")
    print()
    print("Stats:")
    for k in ("frames_total", "meas_epoch", "meas_extra", "ext_sensor_meas",
              "passthrough", "main_subblocks", "aux1_subblocks",
              "aux2_subblocks", "imu_rows", "crc_failures", "extra_no_meas"):
        print(f"  {k:<20} {fmt(stats[k])}")

    if stats["aux2_subblocks"]:
        print(f"\nNote: dropped {stats['aux2_subblocks']} Type1 sub-blocks "
              f"with AntennaID >= 2 (Aux2 not requested).")
    if stats["extra_no_meas"]:
        print(f"\nWarning: {stats['extra_no_meas']} MeasExtra frames had no "
              f"matching preceding MeasEpoch (same TOW) and were dropped.")
    if main_bad or aux1_bad:
        print(f"\nERROR: output CRC verification failed "
              f"({main_bad} main, {aux1_bad} aux1). The split logic has a bug.")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv) or 0)
