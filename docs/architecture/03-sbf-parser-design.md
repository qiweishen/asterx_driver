# 03 — SBF Parser Design

**Status**: Draft for Phase 2A
**Owner**: cpp-architect (spec); ethernet-protocol-expert (Phase 3 implementation)
**Depends on**: `00-requirements.md` (FROZEN, signed 2026-05-16), `02-threading-model.md`

## Traceability

- **Q2** (data products: ExtSensorMeas, MeasEpoch, MeasExtra, AttEuler, AttCovEuler, AuxAntPositions, ReceiverStatus, QualityInd) → §3 block dispatch table.
- **Q9** (timestamp fidelity: preserve TOW/WNc as-is) → §6 timestamp handling rule.
- **Q11** (driver pushes receiver config; CRC checks are nonetheless mandatory) → §2 CRC strategy.
- **Q18** (parser must be testable in isolation) → §8 parser interface.

---

## 1. SBF frame format reference

Every SBF block on the wire conforms to this layout (little-endian throughout):

```
Offset Size   Field         Description
------ -----  -----------   ---------------------------------------------------
   0     2    SyncBytes     ASCII "$@" (0x24, 0x40)
   2     2    CRC           CRC-CCITT-XMODEM over bytes [4 .. 4+Length-1)
   4     2    ID            uint16_t — block_id in low 13 bits, rev in high 3
   6     2    Length        uint16_t — total block size, MUST be multiple of 4
   8     N    Body          block-specific payload (Length - 8 bytes)
```

References:

- `docs/AsteRx-i3_D_Pro__Firmware_v1_5_2_Reference_Guide.pdf`, "Service Block Format" section
- `.claude/skills/septentrio-sbf-protocol/SKILL.md` (auto-loaded)

Three invariants the parser depends on:

1. **`Length` is total**: includes header (8 bytes) and body. So `body_size = Length - 8`.
2. **`Length` is a multiple of 4**: receiver pads bodies to 4-byte alignment with reserved bytes. Parser must NOT trust that the "useful" content ends at byte (Length - 8); some blocks declare a `subblock_count` and the trailing padding is meaningless.
3. **`CRC` is computed over bytes 4..(Length-1)** — i.e. starting with the `ID` field, **not** including `SyncBytes` and **not** including the `CRC` field itself.

---

## 2. CRC algorithm — CRC-CCITT-XMODEM

**Parameters** (per AsteRx manual, "CRC Computation"):

| Parameter | Value |
|---|---|
| Polynomial | 0x1021 |
| Initial value | 0x0000 |
| Reflect input | No |
| Reflect output | No |
| XOR output | 0x0000 |

This is the **XMODEM** flavor of CRC-CCITT, often called CRC-16/XMODEM. **NOT** the same as "CRC-CCITT-FALSE" (init = 0xFFFF) which is more common in hobbyist code and would silently produce wrong CRCs.

### 2.1 Algorithm

Reference implementation, bit-serial (for correctness reference; production code uses table-lookup):

```cpp
constexpr uint16_t crcCcittXmodemBitwise(std::span<const uint8_t> data) noexcept {
  uint16_t crc = 0x0000;
  for (uint8_t b : data) {
    crc ^= static_cast<uint16_t>(b) << 8;
    for (int i = 0; i < 8; ++i) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}
```

### 2.2 Table-driven implementation (production)

256-entry table, pre-computed at compile time via `constexpr`. Each table entry is the CRC of a single byte starting from CRC = (byte << 8).

```cpp
// At namespace scope in include/asterx/sbf/crc.hpp:
namespace asterx::sbf {

inline constexpr auto kCrcTable = [] {
  std::array<uint16_t, 256> t{};
  for (int i = 0; i < 256; ++i) {
    uint16_t c = static_cast<uint16_t>(i) << 8;
    for (int j = 0; j < 8; ++j) {
      c = (c & 0x8000) ? static_cast<uint16_t>((c << 1) ^ 0x1021)
                       : static_cast<uint16_t>(c << 1);
    }
    t[i] = c;
  }
  return t;
}();

constexpr uint16_t crcCcittXmodem(std::span<const uint8_t> data) noexcept {
  uint16_t crc = 0x0000;
  for (uint8_t b : data) {
    crc = (crc << 8) ^ kCrcTable[(crc >> 8) ^ b];
  }
  return crc;
}

}  // namespace asterx::sbf
```

A unit test fixture must include:

- The all-zeros input (CRC = 0x0000).
- A known short sequence with hand-computed CRC.
- A captured SBF frame from the receiver (verifies the *byte range* is correct as well as the math).

### 2.3 What the CRC covers

```
bytes:    [0][1][2][3][4][5][6][7][8] ... [Length-1]
          | sync |  CRC | ID  | Len |  body ...
          ^^^^^^^^^^^^^^                                 NOT covered
                 ^^^^^                                  NOT covered (CRC field itself)
                       ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^   <-- CRC covers exactly this range
```

In code: `crcCcittXmodem(frame_bytes.subspan(4, length - 4))`.

**This is the single most common mistake in third-party SBF parsers** (including a now-fixed bug in early versions of `septentrio_gnss_driver`): including the sync or the CRC field itself. The protocol expert must add a regression test against a captured real-receiver frame.

---

## 3. Block dispatch table

Block IDs we implement (sorted by block_id). All IDs are the **low 13 bits** of the `ID` field; the high 3 bits are the revision (`rev`) which we record but do not use for dispatch.

| block_id | Name | SBF Manual § | Expected rate | Body size (approx) | Priority |
|---|---|---|---|---|---|
| 4006 | MeasEpoch | 3.4.1 | 10 Hz | variable, dominated by sub-blocks (≈ 600–900 B at 30 SVs) | P0 (must) |
| 4014 | ReceiverStatus | 3.5.1 | 1 Hz | ~ 100 B fixed | P1 (must) |
| 4027 | MeasExtra | 3.4.1 | 10 Hz | variable, ≈ 12 B × N_sub | P0 (must) |
| 4050 | ExtSensorMeas | 3.4.2 | 200 Hz | 32 B fixed (one IMU sample per block) | P0 (must) |
| 5938 | AttEuler | 3.3.4 | 10 Hz | 36 B fixed | P0 (must) |
| 5939 | AttCovEuler | 3.3.4 | 10 Hz | 36 B fixed | P0 (must) |
| 5942 | AuxAntPositions | 3.3.5 | 10 Hz | variable, ≈ 78 B × N_sub | P0 (must) |
| 4082 | QualityInd | 3.5.1 | 1 Hz | ~ 20 B fixed | P1 (must) |
| anything else | — | — | — | — | passthrough as `UnknownBlock` |

Block IDs were validated against `docs/AsteRx-i3_D_Pro__Firmware_v1_5_2_Reference_Guide.pdf`; the protocol expert MUST re-verify each ID against the manual section cited above before writing the parser, because off-by-one drift between firmware versions is a known footgun. If the manual disagrees with this table, **manual wins**; surface as a Phase 3 spec amendment.

### 3.1 Dispatch implementation

A static table indexed by truncated block_id, or a `std::unordered_map<uint16_t, FrameHandler>`. Given there are only ~ 8 known IDs and we want fast lookup with no map overhead, prefer a switch statement inside `FrameDispatcher::dispatch`:

```cpp
std::optional<RecordEnvelope> FrameDispatcher::dispatch(const Frame& f) {
  switch (f.blockId()) {
    case 4006: return buildMeasEpoch(f);
    case 4014: return buildReceiverStatus(f);
    case 4027: return buildMeasExtra(f);
    case 4050: return buildExtSensorMeas(f);
    case 5938: return buildAttEuler(f);
    case 5939: return buildAttCovEuler(f);
    case 5942: return buildAuxAntPositions(f);
    case 4082: return buildQualityInd(f);
    default:   return buildUnknown(f);
  }
}
```

Switch is faster than the map, the compiler likely emits a jump table, and adding a new block type is a one-liner.

### 3.2 Unknown block policy

The handler `buildUnknown` constructs an `UnknownBlock` envelope:

```cpp
struct UnknownBlock {
  uint16_t block_id;
  uint16_t rev;
  uint32_t tow_ms;
  uint16_t wnc;
  std::vector<std::uint8_t> raw_body;  // size = Length - 8
};
```

The recording layer writes it as an opaque blob (with the recorded `block_id`) so future versions of the reader can decode it once the schema is known.

Per `debug.emit_unknown_block_stats_every` (default 100), the parser logs a debug message every Nth occurrence per `block_id`:

```cpp
auto& count = unknownBlockCounts_[block_id];
if (++count % emit_every_ == 1) {
  spdlog::debug("unknown SBF block id={} count={} body_len={}",
                block_id, count, length - 8);
}
```

---

## 4. Frame synchronization state machine

The parser is fed an arbitrary byte stream; framing isn't aligned with TCP read boundaries. We implement an explicit state machine:

```
                       any byte != '$'
                +-------------------------------+
                v                               |
            +-------+    '$'   +-----------+    |
   ---->    | IDLE  |--------->| GOT_DOLLAR|----+
            +-------+          +-----------+
                                  |     |
                                  | '@' | other byte
                                  v     |
                              +---------+----+
                              | GOT_SYNC      |
                              +---------------+
                                  | next 2 bytes
                                  v
                              +---------------+
                              | READING_CRC   |
                              +---------------+
                                  | next 2 bytes
                                  v
                              +---------------+
                              | READING_ID    |
                              +---------------+
                                  | next 2 bytes
                                  v
                              +---------------+
                              | READING_LEN   |
                              +---------------+
                                  | (Length - 8) bytes
                                  v
                              +---------------+
                              | READING_BODY  |
                              +---------------+
                                  | CRC match?
                          +-------+-------+
                          | yes           | no
                          v               v
                  emit Frame      [resync: search past sync for next '$']
                          |               |
                          +-------+-------+
                                  v
                              +-------+
                              | IDLE  |
                              +-------+
```

### 4.1 Resync rule

When the CRC check fails or `Length` is invalid (not a multiple of 4, or > max SBF size 65535 - we use 8188 as a softer cap matching the receiver's actual max):

1. Increment `crc_failed_count` (or `length_invalid_count`).
2. **Do NOT** simply drop the whole buffer. Drop the first sync byte and rescan the *remainder of the already-buffered bytes* for the next `$@` pair. This handles back-to-back corrupt frames without losing the start of a good frame that happens to begin in the middle of a corrupt one.
3. If no further `$@` is found in the buffered window, return IDLE and continue with new bytes.

### 4.2 Maximum frame size

Per the AsteRx-i3 manual, no SBF block exceeds 8188 bytes (some MeasEpoch with very large sub-block lists can approach this). Our parser MUST cap `Length` at 8188; anything larger is corruption.

```cpp
inline constexpr uint16_t kMaxSbfLength = 8188;
```

A length > 8188 is treated as a sync error (resync). Otherwise an attacker (or a misconfigured intermediate proxy) could DoS us with declared lengths up to 65532.

### 4.3 State persistence across feed() calls

`SbfFrameSync` holds an internal byte buffer (a deque of bytes, capacity 8188 + some slack). `feed(std::span<const uint8_t>)` appends to the buffer and runs the state machine until either:
- A complete frame is emitted (pushed to `out` vector).
- The buffer is exhausted without completing a frame.

The buffer can grow up to `kMaxSbfLength`; the parser never holds more than one in-progress frame. Beyond `kMaxSbfLength` we are in the "no sync found" state and trim the buffer to keep only the last ~ 8 bytes (so we can still pick up a `$@` straddling two feeds).

---

## 5. Sub-block walking (MeasEpoch, MeasExtra, AuxAntPositions)

This is the second-most-common bug surface in SBF parsers. Several blocks have variable bodies declared in their header:

### 5.1 MeasEpoch (block_id 4006) sub-block layout

```
Fixed header (16 bytes):
  TOW (u32)
  WNc (u16)
  N (u8)            -- number of Type-1 sub-blocks
  SB1Length (u8)    -- size of each Type-1 sub-block (in bytes) (** dynamic! **)
  SB2Length (u8)    -- size of each Type-2 sub-block (in bytes) (** dynamic! **)
  CommonFlags (u8)
  CumClkJumps (u8)
  Reserved (u8)
  N_Type1 (alias for N)
  ... (per-block additional header fields per manual)

For i in 0..N-1:
  TypeOne_i:    SB1Length bytes
    (within TypeOne_i: at offset 1: N_2 (u8) -- count of Type-2 sub-blocks under this Type-1)
    For j in 0..N_2-1:
      TypeTwo_ij: SB2Length bytes
```

**The pitfall**: many third-party parsers hardcode `SB1Length` / `SB2Length` as compile-time constants taken from the manual. The receiver firmware is permitted to make these *bigger* in later versions (adding new fields at the end of each sub-block). The driver MUST read these from the header byte and `memcpy` only the fields it knows, treating the trailing bytes (if `SB1Length > sizeof(known_fields)`) as forward-compatible padding.

### 5.2 The walking template

```cpp
// Pseudocode for MeasEpoch
const uint8_t* p = frame.body();
const auto fixed = readFixedHeader(p);
p += fixed.fixedHeaderSize();   // e.g. 16 bytes
const std::uint8_t N      = fixed.n;
const std::uint8_t sb1Len = fixed.sb1Length;
const std::uint8_t sb2Len = fixed.sb2Length;

if (sb1Len < kMinSb1Size || sb2Len < kMinSb2Size) {
  bumpCounter("measepoch_sb_too_small");
  return std::nullopt;  // refuse to parse
}

for (std::uint8_t i = 0; i < N; ++i) {
  if (p + sb1Len > frame.bodyEnd()) { bumpCounter("measepoch_truncated"); break; }
  auto sb1 = readSb1Known(p);  // memcpys only the fields we know; trailing bytes ignored.
  p += sb1Len;

  const std::uint8_t n2 = sb1.n_type2;
  for (std::uint8_t j = 0; j < n2; ++j) {
    if (p + sb2Len > frame.bodyEnd()) { bumpCounter("measepoch_truncated"); break; }
    auto sb2 = readSb2Known(p);
    p += sb2Len;
    sb1.sb2.push_back(sb2);
  }
  out.sb1.push_back(std::move(sb1));
}
```

The same template applies to `MeasExtra` (one level of sub-blocks) and `AuxAntPositions` (one level of sub-blocks).

### 5.3 `kMinSb1Size` / `kMinSb2Size`

These are the **manual-specified** minimum sizes — what the firmware promises will always be present. We refuse to parse if `SB1Length` is smaller (a sign of corruption or a firmware downgrade scenario), but we permit it to be larger.

For MeasEpoch as of firmware v1.5.2 manual:
- `kMinSb1Size = 20` (Type-1 sub-block)
- `kMinSb2Size = 12` (Type-2 sub-block)

The protocol expert MUST re-verify these from the manual when implementing.

---

## 6. Don't-Use sentinels and `std::optional` fields

Septentrio uses dedicated bit patterns to mean "this field is not available in the current epoch". The parser MUST convert these to `std::nullopt`.

| Type | Bit pattern (hex) | Meaning |
|---|---|---|
| u1 / u8 | 0xFF | Don't use |
| u2 / u16 | 0xFFFF | Don't use |
| u4 / u32 | 0xFFFFFFFF | Don't use |
| u8 / u64 | 0xFFFFFFFFFFFFFFFF | Don't use |
| i1 / i8 | 0x80 (= -128) | Don't use |
| i2 / i16 | 0x8000 | Don't use |
| i4 / i32 | 0x80000000 | Don't use |
| i8 / i64 | 0x8000000000000000 | Don't use |
| f4 / float | NaN payload 0x7FC00000 | Don't use |
| f8 / double | NaN payload 0x7FF8000000000000 | Don't use |

### 6.1 Helper macros / templates

```cpp
namespace asterx::sbf {

template <class T>
constexpr T kDoNotUseSentinel = /* overload per type */;

template <> inline constexpr uint8_t  kDoNotUseSentinel<uint8_t>  = 0xFF;
template <> inline constexpr uint16_t kDoNotUseSentinel<uint16_t> = 0xFFFF;
template <> inline constexpr uint32_t kDoNotUseSentinel<uint32_t> = 0xFFFFFFFFu;
template <> inline constexpr uint64_t kDoNotUseSentinel<uint64_t> = 0xFFFFFFFFFFFFFFFFull;
template <> inline constexpr int8_t   kDoNotUseSentinel<int8_t>   = static_cast<int8_t>(0x80);
// ... etc.

template <class T>
constexpr bool isDoNotUse(T v) noexcept {
  if constexpr (std::is_floating_point_v<T>) {
    return std::isnan(v); // both f4 and f8 sentinels are quiet NaNs; any NaN reads as DNU
  } else {
    return v == kDoNotUseSentinel<T>;
  }
}

template <class T>
constexpr std::optional<T> readOpt(T v) noexcept {
  return isDoNotUse(v) ? std::nullopt : std::optional{v};
}

}  // namespace asterx::sbf
```

Per-block builders use `readOpt` for every field that the manual flags as DNU-capable. The result is each `RecordEnvelope` exposes optional fields exactly mirroring the SBF semantics.

### 6.2 Timestamp DNU handling (Q9)

`TOW` (u32, ms) and `WNc` (u16) can both be DNU. Rule:

- **If TOW is DNU**: record `tow_ms = std::nullopt` in the envelope; the recording header field still gets written (we record DNU as 0xFFFFFFFF for byte-fidelity), but a flag `tow_authoritative = false` is set.
- **NEVER substitute a wall-clock estimate**. The user (a researcher) explicitly wants to know when the receiver lost its TOW.

Sample alignment with wall-clock: we capture `unix_ns` = our local `std::chrono::system_clock::now()` as a **secondary** timestamp in the record envelope, with a flag `unix_ns_authoritative` (which is *always false* for now — we don't have NTP-disciplined wall-clock; user can enable later if/when they pull in chrony/PTP).

---

## 7. Endianness — explicit memcpy with std::endian guard

SBF is little-endian. x86_64 is little-endian. On x86_64 the trivial `reinterpret_cast<const uint32_t*>(p)` works **and is undefined behavior** (it violates pointer aliasing rules; sanitizers catch it; future ARM cross-compile will break it).

**Rule**: every multi-byte field read goes through `std::memcpy` into a local typed variable.

```cpp
template <class T>
T readLe(const uint8_t* p) noexcept {
  static_assert(std::is_trivially_copyable_v<T>);
  T v;
  std::memcpy(&v, p, sizeof(T));
  if constexpr (std::endian::native != std::endian::little) {
    v = byteswap(v);  // future-proof for ARM big-endian
  }
  return v;
}
```

Compilers (GCC 9+, Clang 12+) optimize `memcpy(sizeof <= 8)` to a single MOV; performance is identical to the UB version, and we get well-defined behavior.

`std::endian` is a C++20 feature. On C++17 (our target — Q24), we use a fallback:

```cpp
#if __cplusplus >= 202002L
  #include <bit>
  using std::endian;
#else
  enum class endian { little = 1234, big = 4321,
                      native = (__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__) ? little : big };
#endif
```

(`__BYTE_ORDER__` is set by GCC/Clang; if Apple Clang on some embedded toolchain doesn't, we fall back to a compile-time test.)

---

## 8. Parser public interface (initial signatures)

These live in `include/asterx/sbf/`. Headers are owned by the architect; bodies by the protocol expert.

### 8.1 `frame.hpp`

```cpp
namespace asterx::sbf {

class Frame {
 public:
  uint16_t crc()       const noexcept { return crc_; }
  uint16_t rawId()     const noexcept { return raw_id_; }
  uint16_t blockId()   const noexcept { return raw_id_ & 0x1FFFu; }
  uint8_t  blockRev()  const noexcept { return static_cast<uint8_t>(raw_id_ >> 13); }
  uint16_t length()    const noexcept { return length_; }
  std::span<const uint8_t> body() const noexcept { return body_; }
  const uint8_t* bodyEnd() const noexcept { return body_.data() + body_.size(); }

 private:
  Frame(uint16_t crc, uint16_t raw_id, uint16_t length, std::span<const uint8_t> body)
    : crc_{crc}, raw_id_{raw_id}, length_{length}, body_{body} {}
  uint16_t crc_;
  uint16_t raw_id_;
  uint16_t length_;
  std::span<const uint8_t> body_;
  friend class SbfFrameSync;
};

}  // namespace asterx::sbf
```

### 8.2 `frame_sync.hpp`

```cpp
namespace asterx::sbf {

struct FrameSyncStats {
  uint64_t bytes_consumed{};
  uint64_t frames_emitted{};
  uint64_t crc_failed{};
  uint64_t length_invalid{};
  uint64_t resyncs{};
};

class SbfFrameSync {
 public:
  /// Feed an arbitrary byte chunk. Emits zero or more frames into `out`.
  /// The Frame's body span is valid until the next call to feed() or the
  /// destruction of this object.
  void feed(std::span<const uint8_t> bytes, std::vector<Frame>& out);

  const FrameSyncStats& stats() const noexcept { return stats_; }

  /// Discard any in-progress frame (used during reconnect).
  void reset() noexcept;

 private:
  enum class State { Idle, GotDollar, ReadingHeader, ReadingBody };
  std::deque<uint8_t> buf_;
  State state_{State::Idle};
  FrameSyncStats stats_{};
};

}  // namespace asterx::sbf
```

### 8.3 `frame_dispatcher.hpp`

```cpp
namespace asterx::sbf {

class FrameDispatcher {
 public:
  /// Returns an envelope on success, or std::nullopt if the dispatcher
  /// chose not to materialize one (e.g. unknown block being filtered out).
  /// In the current design we always materialize (UnknownBlock catch-all),
  /// so std::nullopt is reserved for future filtering.
  std::optional<RecordEnvelope> dispatch(const Frame& f) noexcept;

  const FrameDispatchStats& stats() const noexcept { return stats_; }

 private:
  FrameDispatchStats stats_{};
  std::unordered_map<uint16_t, uint64_t> unknown_counts_;
};

}  // namespace asterx::sbf
```

### 8.4 Stats observable to tests

`FrameSyncStats` and `FrameDispatchStats` are part of the public-test surface: tests assert that `crc_failed == 1` after feeding a deliberately corrupted frame, etc.

---

## 9. Error injection hooks for tests

To make the parser testable in isolation:

1. **The parser takes byte spans, never sockets**. Tests feed `std::vector<uint8_t>` literals.
2. **Stats are observable** via const ref accessors (no atomic in this layer; everything runs on the parser thread).
3. **Frame is move-only**, but tests can construct a `Frame` only via `SbfFrameSync` to avoid faking up invalid frames in tests. (The protocol expert may add a `friend` test helper if needed.)
4. **Fixtures**: `tests/fixtures/sbf/` will contain captured raw frames (one per known block_id), plus deliberately corrupted variants (bad CRC, bad length, truncated body, oversize length).

---

## 10. What the SBF parser does NOT do

- It does not open sockets.
- It does not call into the recorder.
- It does not log to spdlog directly (it bumps counters; the parser thread's outer loop logs periodically based on counter deltas). This keeps the parser allocation-free and lock-free.
- It does not allocate per-frame: the `Frame` body span points into the parser's internal `std::deque<uint8_t>` buffer; the frame's lifetime is "until the next `feed()` call". Builders that need to outlive that copy their data into the `RecordEnvelope` (which itself goes into the SPSC queue with move semantics).

---

## Open questions for implementation

1. **`std::deque<uint8_t>` vs ring-buffer for in-progress frame buffer**: deque is allocation-noisy at the per-byte level. Recommendation: implement a fixed-capacity ring buffer of 16 KiB (2× max SBF length, so we always have one full frame's worth of head-room without resizing). Protocol expert decides exact layout.
2. **Whether `Frame::body()` should be `std::span<const uint8_t>` or a `BodyView` class with helpers** (`readU8(offset)`, `readF8(offset)`). The latter is more ergonomic for the dispatcher; the former is simpler. Recommendation: start with `span`, add helpers if call sites get noisy.
3. **Whether to materialize MeasEpoch sub-blocks eagerly into vectors** or keep them as lazy views over the body buffer. Eager copying is simpler and matches the "envelope is self-contained" property; lazy is faster but couples Frame and Envelope lifetimes. Recommendation: eager, with `boost::container::small_vector<Sub, 16>` to keep typical SV counts inline.
4. **Whether to read `block_id` and `length` via `Frame` accessors only, or expose `Frame::header()` for raw access**: tests sometimes want raw access. Recommendation: add a `Frame::rawHeader() -> array<uint8_t, 8>` for tests; keep typed accessors for production.
5. **Validating MeasEpoch fixed-header size against `SBLength` arithmetic**: the manual specifies a fixed header size, but per §5 we allow firmware to grow it. Decision needed on the *minimum* fixed-header size — Phase 3 spec amendment if the manual is ambiguous.
