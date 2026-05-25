#!/usr/bin/env python3
"""Decode a recorded .sbf file and print a per-block summary.

Usage:
    python scripts/parser.py [path/to/recording.sbf]

Default path: recordings/asterx-20260522T055234Z-1.sbf
"""
import struct
import sys
from collections import defaultdict

import sbf_parser

DEFAULT_PATH = "recordings/asterx-20260522T055234Z-1.sbf"

# Septentrio SBF block-ID → canonical name. Lets us identify "Unknown" /
# "BadSBF" entries that this particular sbf_parser build doesn't decode.
# Source: AsteRx-i3 D Pro+ Firmware Reference Guide, SBF block catalog.
SBF_ID_NAMES = {
    4014: "ReceiverStatus",
    4000: "MeasExtra",
    4027: "MeasEpoch",
    4050: "ExtSensorMeas",
    4082: "QualityInd",
    5902: "ReceiverSetup",
    5914: "ReceiverTime",
    5922: "EndOfMeas",
    5938: "AttEuler",
    5939: "AttCovEuler",
    5942: "AuxAntPositions",
    5943: "EndOfAtt",
    4007: "PVTCartesian",
    4006: "PVTGeodetic",
}


def block_id(block_desc, infos):
    """Best-effort extraction of the numeric SBF block ID (13-bit value)."""
    for src in (block_desc, infos):
        for attr in ("id", "block_id", "ID", "BlockID", "block_num", "blockNumber"):
            v = getattr(src, attr, None)
            if v is None and isinstance(src, dict):
                v = src.get(attr)
            if isinstance(v, int):
                return v & 0x1FFF   # mask off the 3-bit revision field
    return None


# ---------------------------------------------------------------------------
# Pure-stdlib raw SBF frame walker. Used to ground-truth the sbf_parser
# library's classification: we walk $@ sync, validate the CRC-CCITT-XMODEM,
# and tally each block ID exactly as the receiver emitted it.
# ---------------------------------------------------------------------------

def crc_ccitt_xmodem(data):
    """CRC-16/XMODEM: poly=0x1021, init=0x0000, no reflection."""
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def raw_scan(path):
    """Walk the file byte-by-byte, validating CRCs. On CRC mismatch the scan
    trusts the length field and jumps ahead — if the next bytes are not '$@'
    we fall back to one-byte step. Returns a dict-of-counts.
    """
    with open(path, "rb") as f:
        data = f.read()

    id_counts          = defaultdict(int)
    rev_counts         = defaultdict(int)
    crc_failed_by_id   = defaultdict(int)
    total_bytes        = 0
    frames             = 0
    crc_fails_total    = 0
    file_size          = len(data)

    i = 0
    n = file_size
    while i + 8 <= n:
        if data[i] != 0x24 or data[i + 1] != 0x40:    # SBF sync = '$', '@'
            i += 1
            continue
        crc, raw_id, length = struct.unpack_from("<HHH", data, i + 2)
        bid = raw_id & 0x1FFF
        rev = (raw_id >> 13) & 0x7
        if length < 8 or length > 8188 or length % 4 != 0 or i + length > n:
            i += 1
            continue
        computed = crc_ccitt_xmodem(data[i + 4 : i + length])
        if computed != crc:
            crc_fails_total += 1
            crc_failed_by_id[bid] += 1
            # Trust the length; if the next bytes are also a sync header, it's
            # almost certainly a real frame boundary that the CRC algorithm
            # disagrees on (probably a parser bug); jump past it. Otherwise
            # fall back to byte-step.
            if i + length + 2 <= n and data[i + length] == 0x24 and data[i + length + 1] == 0x40:
                i += length
            else:
                i += 1
            continue
        id_counts[bid]  += 1
        rev_counts[bid]  = max(rev_counts[bid], rev)
        total_bytes     += length
        frames          += 1
        i += length

    return {
        "id_counts":          id_counts,
        "rev_counts":         rev_counts,
        "crc_failed_by_id":   crc_failed_by_id,
        "total_bytes":        total_bytes,
        "frames":             frames,
        "crc_fails":          crc_fails_total,
        "file_size":          file_size,
        "head_hex":           data[:32].hex(),
    }


def print_raw_summary(path):
    r = raw_scan(path)
    print(f"\n=== Raw-byte scan (independent of sbf_parser) ===")
    print(f"File: {path}  ({r['file_size']:,} bytes)")
    print(f"First 32 bytes: {r['head_hex']}")
    print(f"Frames (CRC-valid): {r['frames']}   Recognized bytes: {r['total_bytes']:,}")
    print(f"CRC failures: {r['crc_fails']}   "
          f"Unrecognized bytes: {r['file_size'] - r['total_bytes']:,}")
    print()
    print(f"  {'BlockID':>8}  {'Rev':>3}  {'Name':<22}  {'OK count':>10}  {'CRC fails':>10}")
    all_ids = sorted(set(r["id_counts"]) | set(r["crc_failed_by_id"]),
                     key=lambda b: -(r["id_counts"].get(b, 0) + r["crc_failed_by_id"].get(b, 0)))
    for bid in all_ids:
        name = SBF_ID_NAMES.get(bid, "(unmapped ID)")
        ok   = r["id_counts"].get(bid, 0)
        bad  = r["crc_failed_by_id"].get(bid, 0)
        rev  = r["rev_counts"].get(bid, 0)
        print(f"  {bid:>8}  {rev:>3}  {name:<22}  {ok:>10}  {bad:>10}")


def name_of(block_desc):
    """Coerce a block descriptor into a string key, robust to library API."""
    for attr in ("name", "block_name", "Name"):
        v = getattr(block_desc, attr, None)
        if v is not None:
            return str(v)
    return str(block_desc)


def field(obj, *names):
    """Return the first available attribute / key from `obj`."""
    for n in names:
        if hasattr(obj, n):
            v = getattr(obj, n)
            if v is not None:
                return v
        if isinstance(obj, dict) and n in obj:
            return obj[n]
    return None


def antenna_ids(infos):
    """Best-effort extraction of AntennaIDs present in a MeasEpoch block.

    SBF MeasEpoch carries one or more Type1 sub-blocks, each tagged with an
    AntennaID (0 = Main, 1 = Aux1, 2 = Aux2). Different sbf_parser builds
    expose this either as a flat list of sub-blocks or as a single struct.
    """
    aids = []
    # case 1: list-like of sub-blocks
    for attr in ("Type1", "type1", "sub_blocks", "SubBlocks", "channels"):
        sub = getattr(infos, attr, None)
        if sub is None and isinstance(infos, dict):
            sub = infos.get(attr)
        if sub is None:
            continue
        try:
            for sb in sub:
                aid = field(sb, "AntennaID", "antenna_id", "AntID")
                if aid is not None:
                    aids.append(int(aid))
        except TypeError:
            pass
    # case 2: scalar field on the block itself
    if not aids:
        aid = field(infos, "AntennaID", "antenna_id", "AntID")
        if aid is not None:
            aids.append(int(aid))
    return aids


def summarize(path):
    counts            = defaultdict(int)
    first_tow         = {}
    last_tow          = {}
    antenna_counts    = defaultdict(int)
    bad_tow           = 0
    # block-ID histograms for unparsed / bad blocks
    unknown_by_id     = defaultdict(int)
    bad_by_id         = defaultdict(int)

    for block_desc, infos in sbf_parser.read(path):
        name = name_of(block_desc)
        counts[name] += 1

        if "Unknown" in name or "BadSBF" in name:
            bid = block_id(block_desc, infos)
            if "Unknown" in name:
                unknown_by_id[bid] += 1
            else:
                bad_by_id[bid] += 1

        tow = field(infos, "TOW", "tow")
        if isinstance(tow, (int, float)):
            if int(tow) == 0xFFFFFFFF:
                bad_tow += 1
            else:
                first_tow.setdefault(name, tow)
                last_tow[name] = tow

        if "MeasEpoch" in name:
            for aid in antenna_ids(infos):
                antenna_counts[aid] += 1

    total = sum(counts.values())
    print(f"\nFile: {path}")
    print(f"Total blocks decoded: {total}")
    if bad_tow:
        print(f"Blocks with DNU TOW (0xFFFFFFFF): {bad_tow}")
    print()
    print(f"{'Block':<28} {'Count':>10} {'~ Hz':>8} {'Duration (s)':>14}")
    print("-" * 64)
    for name, count in sorted(counts.items(), key=lambda x: -x[1]):
        ft = first_tow.get(name)
        lt = last_tow.get(name)
        if ft is not None and lt is not None and lt > ft:
            duration_s = (lt - ft) / 1000.0          # SBF TOW is in ms
            hz = count / duration_s if duration_s > 0 else float("nan")
            print(f"{name:<28} {count:>10} {hz:>8.2f} {duration_s:>14.2f}")
        else:
            print(f"{name:<28} {count:>10} {'n/a':>8} {'n/a':>14}")

    if unknown_by_id or bad_by_id:
        print("\n'Unknown' / 'BadSBF' breakdown by block ID:")
        print(f"  {'BlockID':>8}  {'Probable name':<22}  {'Unknown':>10}  {'BadSBF':>10}")
        ids = sorted(set(unknown_by_id) | set(bad_by_id),
                     key=lambda x: (x is None, x))
        for bid in ids:
            probable = SBF_ID_NAMES.get(bid, "(unknown ID)" if bid is not None else "(no ID extractable)")
            u = unknown_by_id.get(bid, 0)
            b = bad_by_id.get(bid, 0)
            bid_str = str(bid) if bid is not None else "?"
            print(f"  {bid_str:>8}  {probable:<22}  {u:>10}  {b:>10}")

    if antenna_counts:
        print("\nMeasEpoch sub-blocks by AntennaID:")
        labels = {0: "Main", 1: "Aux1", 2: "Aux2"}
        for aid, c in sorted(antenna_counts.items()):
            label = labels.get(aid, f"AntID={aid}")
            print(f"  {label:<10} {c:>10}")
    else:
        print("\n(No AntennaID information could be extracted from MeasEpoch.)")


# ---------------------------------------------------------------------------
# MeasEpoch v2 decoder. The receiver packs measurements from BOTH antennas
# into a single MeasEpoch block; each Type1 sub-block carries an AntennaID
# in its ObsInfo byte (bits 0-2; 0=Main, 1=Aux1, 2=Aux2).
# ---------------------------------------------------------------------------

# Septentrio signal-type codes per AsteRx-i3 D Pro+ FW Reference §4.1.10.
# SigIdxLo lives in Type byte bits 0-4. If SigIdxLo == 31 the actual signal
# number lives in ObsInfo bits 3-7 with a +32 offset (not handled here; no
# signal in our data hits the extension case).
SIGNAL_TYPE_NAMES = {
    0:  "GPS L1CA",    1:  "GPS L1P",     2:  "GPS L2P",   3:  "GPS L2C",
    4:  "GPS L5",      5:  "GPS L1C",
    6:  "QZS L1CA",    7:  "QZS L2C",
    8:  "GLO L1CA",    9:  "GLO L1P",     10: "GLO L2P",   11: "GLO L2CA",
    12: "GLO L3",
    13: "BDS B1C",     14: "BDS B2a",
    15: "NavIC L5",
    17: "GAL E1",      19: "GAL E6",      20: "GAL E5a",   21: "GAL E5b",
    22: "GAL E5 AltBOC",
    23: "LBand",
    24: "SBAS L1CA",   25: "SBAS L5",
    26: "QZS L5",      27: "QZS L6",
    28: "BDS B1I",     29: "BDS B2I",     30: "BDS B3I",
    32: "QZS L1C",     33: "QZS L1S",
    34: "BDS B2b",
    37: "NavIC L1",
    38: "QZS L1CB",    39: "QZS L5S",
}


def decode_meas_epoch_v2(body):
    """Decode a MeasEpoch v2 block body (bytes AFTER the 8-byte SBF header).
    Returns a list of dicts, one per Type1 sub-block (per-channel observation).
    """
    if len(body) < 12:
        return []
    tow = struct.unpack_from("<I", body, 0)[0]
    wnc = struct.unpack_from("<H", body, 4)[0]
    n1, sb1l, sb2l = body[6], body[7], body[8]
    # CommonFlags, CumClkJumps, Reserved at 9..11

    obs = []
    off = 12
    for _ in range(n1):
        if off + sb1l > len(body):
            break
        rx_ch    = body[off + 0]
        type_b   = body[off + 1]
        sig_t    = type_b & 0x1F            # SignalType — low 5 bits
        antenna  = (type_b >> 5) & 0x07     # AntennaID — high 3 bits of Type
        svid     = body[off + 2]
        cn0_raw  = body[off + 15]
        lock     = struct.unpack_from("<H", body, off + 16)[0]
        n2       = body[off + 19]

        # CN0 unit depends on signal type. For the AsteRx-i3 the formula is:
        #   most signals: CN0_dBHz = 10 + N * 0.25
        #   L5 / E5a / E5b: CN0_dBHz =  N * 0.25
        # We use the +10 form as a reasonable default.
        cn0_dbhz = 10.0 + cn0_raw * 0.25 if cn0_raw != 255 else None

        obs.append({
            "tow":       tow,
            "wnc":       wnc,
            "antenna":   antenna,
            "sig_type":  sig_t,
            "svid":      svid,
            "rx_ch":     rx_ch,
            "cn0_dbhz":  cn0_dbhz,
            "lock_s":    lock,
        })

        off += sb1l + n2 * sb2l
    return obs


def dual_antenna_summary(path):
    """Walk MeasEpoch (block 4027) frames and report per-antenna stats."""
    with open(path, "rb") as f:
        data = f.read()

    obs_count          = defaultdict(int)
    distinct_svs       = defaultdict(set)
    cn0_sum            = defaultdict(float)
    cn0_n              = defaultdict(int)
    signals_per_ant    = defaultdict(lambda: defaultdict(int))   # ant -> sigtype -> count
    epochs             = 0

    i, n = 0, len(data)
    while i + 8 <= n:
        if data[i] != 0x24 or data[i + 1] != 0x40:
            i += 1
            continue
        _, raw_id, length = struct.unpack_from("<HHH", data, i + 2)
        bid = raw_id & 0x1FFF
        if length < 8 or length > 8188 or length % 4 != 0 or i + length > n:
            i += 1
            continue
        if bid == 4027:                         # MeasEpoch
            epochs += 1
            for ob in decode_meas_epoch_v2(data[i + 8 : i + length]):
                a = ob["antenna"]
                obs_count[a] += 1
                distinct_svs[a].add((ob["sig_type"], ob["svid"]))
                if ob["cn0_dbhz"] is not None:
                    cn0_sum[a] += ob["cn0_dbhz"]
                    cn0_n[a]   += 1
                signals_per_ant[a][ob["sig_type"]] += 1
        i += length

    print(f"\n=== Dual-antenna MeasEpoch summary ===")
    print(f"MeasEpoch frames parsed: {epochs}")
    print()
    print(f"  {'Antenna':<8}  {'Obs total':>10}  {'Distinct (sig,SV)':>18}  {'Avg CN0 (dBHz)':>16}")
    labels = {0: "Main", 1: "Aux1", 2: "Aux2"}
    for a in sorted(obs_count):
        avg = cn0_sum[a] / cn0_n[a] if cn0_n[a] else float("nan")
        print(f"  {labels.get(a, f'A{a}'):<8}  {obs_count[a]:>10}  "
              f"{len(distinct_svs[a]):>18}  {avg:>16.2f}")

    if not obs_count:
        print("  (no MeasEpoch frames found)")
        return

    print()
    print("Signal-type breakdown per antenna:")
    for a in sorted(signals_per_ant):
        label = labels.get(a, f"A{a}")
        for sig_t, c in sorted(signals_per_ant[a].items(), key=lambda x: -x[1]):
            sig_name = SIGNAL_TYPE_NAMES.get(sig_t, f"sig#{sig_t}")
            print(f"  {label:<6} {sig_name:<22} {c:>10}")


# ---------------------------------------------------------------------------
# QualityInd (block 4082) decoder — per AsteRx-i3 D Pro+ FW Reference §4.2.16.
#
# Block body layout (post-8-byte SBF header):
#   bytes 0-3:  TOW (u4)
#   bytes 4-5:  WNc (u2)
#   byte 6:     N (number of indicators)
#   byte 7:     Reserved
#   bytes 8+:   N × u2, each indicator packed as:
#                 bits 0-7:   Indicator type
#                 bits 8-11:  Value (0 = poor, 10 = excellent, 15 = unknown)
#                 bits 12-15: Reserved
# ---------------------------------------------------------------------------

QUALITY_IND_NAMES = {
    0:  "Overall quality",
    1:  "GNSS signals from Main antenna",
    2:  "GNSS signals from Aux1 antenna",
    11: "RF power level — Main antenna",
    12: "RF power level — Aux1 antenna",
    21: "CPU headroom",
    25: "OCXO stability",
    29: "Scintillation score",
    30: "Base-station measurements",
    31: "RTK post-processing prospect",
}


def quality_ind_summary(path):
    with open(path, "rb") as f:
        data = f.read()

    values = defaultdict(list)  # type -> [values]
    frames = 0

    i, n = 0, len(data)
    while i + 8 <= n:
        if data[i] != 0x24 or data[i + 1] != 0x40:
            i += 1
            continue
        _, raw_id, length = struct.unpack_from("<HHH", data, i + 2)
        bid = raw_id & 0x1FFF
        if length < 8 or length > 8188 or length % 4 != 0 or i + length > n:
            i += 1
            continue
        if bid == 4082:                                     # QualityInd
            body = data[i + 8 : i + length]
            if len(body) < 8:
                i += length
                continue
            n_ind = body[6]
            frames += 1
            for k in range(n_ind):
                off = 8 + k * 2
                if off + 2 > len(body):
                    break
                ind = struct.unpack_from("<H", body, off)[0]
                t = ind & 0xFF
                v = (ind >> 8) & 0x0F
                if v != 15:                                 # 15 = unknown
                    values[t].append(v)
        i += length

    print(f"\n=== QualityInd (block 4082) per-indicator summary ===")
    print(f"QualityInd frames parsed: {frames}")
    if not values:
        print("  (no QualityInd frames found)")
        return
    print()
    print(f"  {'Type':>4}  {'Indicator':<35}  {'Samples':>8}  {'Avg':>6}  {'Min':>4}  {'Max':>4}")
    for t in sorted(values):
        v = values[t]
        name = QUALITY_IND_NAMES.get(t, f"(unknown type {t})")
        print(f"  {t:>4}  {name:<35}  {len(v):>8}  {sum(v)/len(v):>6.2f}  {min(v):>4}  {max(v):>4}")


def main(argv):
    path = argv[1] if len(argv) > 1 else DEFAULT_PATH
    print_raw_summary(path)
    dual_antenna_summary(path)
    quality_ind_summary(path)
    print()
    print("=== sbf_parser-based decoded summary ===")
    summarize(path)


if __name__ == "__main__":
    main(sys.argv)
