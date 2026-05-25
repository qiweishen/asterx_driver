# AsteRx-i3 D Pro+ Ethernet Driver — Requirements (APPROVED — FROZEN)

> Status: **READY FOR USER SIGN-OFF**
> All 30 requirement questions answered across 4 batches (Q1–Q30).
> Last updated: 2026-05-16
> Architect: `cpp-architect` agent
> Target: Ubuntu 20.04.6 LTS, x86_64, C++17, **no ROS**, TCP-only (v1).
>
> **To approve**: reply with `approve` (or `批准` or `OK`) and your name as you'd like it recorded.
> **To revise**: indicate which Q# you'd like to change and the new answer.

This document is the single source of truth for what the driver must do. It is built up batch-by-batch through a Q&A loop with the user. The implementer (`ethernet-protocol-expert`) and the reviewer (`cpp-driver-reviewer`) are bound by what is written here.

---

## Batch progress

| Batch | Topic                                | Questions | Status      |
|-------|--------------------------------------|-----------|-------------|
| 1     | Use case & connection model          | Q1–Q8     | ANSWERED    |
| 2     | Custom binary format + receiver cfg + threading | Q9–Q15  | ANSWERED |
| 3     | Errors, logging, config, CLI, credentials, reconnect | Q16–Q22 | ANSWERED  |
| 4     | Testing, build, delivery             | Q23–Q30   | ANSWERED    |

---

## Decision Summary (TL;DR — one line per Q)

| Q | Topic | Decision |
|---|-------|----------|
| Q1 | Primary goal | Offline lossless recording + reserved real-time callback hook |
| Q2 | Raw products | All (IMU + main+aux GNSS raw via MeasEpoch SignalInfo + AttEuler/AttCovEuler + ReceiverStatus/QualityInd) |
| Q3 | Output rates | Config-driven (defaults: IMU 200Hz / MeasEpoch 10Hz / AttEuler 10Hz / ReceiverStatus 1Hz, all overridable) |
| Q4 | On-disk format | Custom binary (NO RxTools compat) |
| Q5 | File rotation | 1 GB OR 1 hour, whichever first |
| Q6 | Transport | Both plaintext 28784 + TLS 28783, config-switchable |
| Q7 | IPS data channel | Yes — separate control connection + dedicated IPS1 data port |
| Q8 | IP discovery | DHCP+mDNS + static IP override (layered) |
| Q9 | Format priority | (1) timestamp fidelity (2) ABI alignment (3) self-descr/versioned (4) mmap decode (5) min size |
| Q10 | Dual-write raw SBF | NO (Format-interface extension point reserved) |
| Q11 | Timestamp | TOW (u32) + WNc (u16) + derived unix_ns (i64, non-authoritative) |
| Q12 | SBF stream alloc | 4 streams (IMU/GNSS/Attitude/Health) → single IPS1 port |
| Q13 | IMU orientation | Config-driven (θX/θY/θZ); default SensorDefault |
| Q14 | Config persistence | Write `current` only; never modify `boot` |
| Q15 | Threading | 3 threads (IO + SBF parser + writer) + boost::lockfree::spsc_queue + callback thread pool |
| Q16 | Errors | Hybrid: tl::expected hot path + asterx::Exception facade; dtors/callbacks never throw |
| Q17 | Logging | Rotating file ONLY (5×100MB, daily, debug level) + startup-window stderr fatal-only exception |
| Q18 | CRC/sync failures | Drop+count+log+watchdog (30s/5% trigger triggers reconnect) |
| Q19 | Config format | TOML via toml++ v3.4.0 + schema doc + example; startup-time strict validation |
| Q20 | CLI | Minimal: `asterx_driver --config <path>` (plus `--log-level`, `--help`, `--version` universal flags only; NO subcommands) |
| Q21 | Credentials | Layered: env → sealed file (mode ≤ 0400) → plaintext+warn → TTY prompt |
| Q22 | Reconnect | Probe-then-decide + 5s data-silence watchdog; full reconfigure on probe mismatch |
| Q23 | Tests | unit + socat integration + replay round-trip; fuzz/property deferred |
| Q24 | Test framework | GoogleTest + GMock (FetchContent v1.14.0) |
| Q25 | Libs | Boost.Asio system / spdlog FC v1.13.0 / toml++ FC v3.4.0 / CLI11 FC v2.4.2 / tl::expected FC v1.1.0 / boost::lockfree system / Avahi system / GTest FC v1.14.0 |
| Q26 | CMake structure | Multi-target monorepo: core STATIC lib + executable + tests + examples |
| Q27 | Sanitizers | 3 options (ASAN/UBSAN/TSAN), CI runs ASAN+UBSAN; TSAN triggers lockfree→mutex swap |
| Q28 | CI | GitHub Actions matrix: 2 OS × 4 compilers × build-types, plus ASAN+UBSAN dedicated job |
| Q29 | Docs | README + arch docs (00–06.md) + USAGE.md + Doxygen on public headers |
| Q30 | Version/ABI | SemVer 0.x; pimpl-based ABI stability post-1.0; namespace asterx::*; no extern "C" v1 |

---

## A. Use case & connection model (Q1–Q8, ANSWERED)

### A1. Primary goal — Q1 = D (default)

**User answer**: Offline lossless recording is the priority; reserve a real-time callback interface for downstream consumers (factor-graph estimator, monitoring tools) without making it a blocking dependency.

**Rationale**: Matches the researcher's RS-NZM-GPCT workflow — record now, post-process or stream into the estimator without re-architecting later.

**Architectural implication**:
- The disk-writer is the canonical sink and must never be blocked by a slow callback.
- Real-time consumers attach via a non-owning `Subscriber` / `Sink` interface; back-pressure on a subscriber must drop frames for *that subscriber only* (with a counter), never for the recorder.
- The pipeline is fan-out: parsed-block → [recorder | subscriber_1 | subscriber_2 | …]; lossless guarantee applies only to the recorder branch.

---

### A2. Raw measurement products — Q2 = D (default, with clarification)

**User answer**: Record all of: IMU acceleration + angular rate, raw GNSS observables from both antennas, GNSS attitude, receiver health.

**Important clarification on "Aux antenna raw observables"**: The AsteRx-i3 D Pro+ exposes both main and Aux1 antenna raw observables through the *same* `MeasEpoch` SBF block. Each `MeasEpochChannelType1` sub-block carries an `AntennaID` field (0 = main, 1 = Aux1, 2 = Aux2). There is **no separate "AuxMeasEpoch" block**. Aux raw obs are obtained by:
1. Configuring the receiver with `setMeasMeasurements all` (so all antennas' measurements are output), and
2. Filtering by `AntennaID` on the consumer side.

`AuxAntPositions` carries only the *baseline* between main and Aux antennas, not raw measurements. We will record it for attitude QC purposes, but it is not a source of raw observables.

**Concrete block list to be recorded** (final list pending Q12 — SBF output stream allocation):

| Block name          | Block number | Purpose                                          |
|---------------------|--------------|--------------------------------------------------|
| `ExtSensorMeas`     | 4050         | IMU accel + gyro (up to 200 Hz)                  |
| `MeasEpoch`         | 4027         | Raw pseudorange / carrier / Doppler / CN0, both antennas |
| `MeasExtra`         | 4000         | Per-signal extra fields (lock time, σ, etc.)     |
| `AuxAntPositions`   | 5942         | Main↔Aux1 baseline                                |
| `AttEuler`          | 5938         | GNSS-derived attitude (yaw/pitch/roll)           |
| `AttCovEuler`       | 5939         | Covariance of `AttEuler`                          |
| `ReceiverStatus`    | 4014         | CPU load, RAM, temperature                       |
| `QualityInd`        | 4082         | Per-signal-source quality indicators              |
| `ReceiverTime`      | 5914         | Receiver clock vs system clock (sanity)          |

Exact block numbers will be reconciled with the Firmware v1.5.2 Reference Guide before the implementer is unblocked.

**Architectural implication**:
- The SBF dispatcher needs to know every block number above. Unknown block numbers must be either (a) logged-and-dropped, or (b) written verbatim to disk — decision pending Q9 (custom format design intent).
- The recorder must preserve `AntennaID` for `MeasEpoch`; it cannot collapse main and Aux into a single stream.

---

### A3. Output rates — Q3 = C (custom, **config-driven**)

**User answer**: All output rates are specified in the driver's config file and pushed to the receiver on connect; nothing hard-coded.

**Rationale**: Different field trials want different IMU rates (low-power 50 Hz, calibration 100 Hz, full 200 Hz); different post-processing pipelines want different GNSS rates. Hard-coding violates reproducibility.

**Recommended default values** (used when config omits a rate):

| Block             | Default `MsgInterval` | SBF token  |
|-------------------|-----------------------|------------|
| `ExtSensorMeas`   | 5 ms (200 Hz)         | `msec5`    |
| `MeasEpoch`       | 100 ms (10 Hz)        | `msec100`  |
| `MeasExtra`       | 100 ms (10 Hz)        | `msec100`  |
| `AuxAntPositions` | 100 ms (10 Hz)        | `msec100`  |
| `AttEuler`        | 100 ms (10 Hz)        | `msec100`  |
| `AttCovEuler`     | 100 ms (10 Hz)        | `msec100`  |
| `ReceiverStatus`  | 1 s   (1 Hz)          | `sec1`     |
| `QualityInd`      | 1 s   (1 Hz)          | `sec1`     |
| `ReceiverTime`    | 1 s   (1 Hz)          | `sec1`     |

**Architectural implication**:
- The driver translates each `block_name → interval_ms` from config into `setSBFOutput` commands at connect time.
- Allowed intervals are constrained by the SBF spec's `MsgInterval` enum (`msec5`, `msec10`, `msec20`, `msec40`, `msec50`, `msec100`, `msec200`, `msec500`, `sec1`, `sec2`, `sec5`, …). Config validation must reject unsupported values with a clear error message.
- The arbiter for "how many SBF streams do we need" is Q12 (pending).

---

### A4. On-disk format — Q4 = C (**custom binary, no RxTools compatibility**)

**User answer**: Use a custom binary format, **not** raw SBF bytes. This is a deliberate trade-off: we lose RxTools / `bin2asc` replay compatibility in exchange for a format tuned to the user's downstream needs.

**This is a deviation from the default recommendation** and the most architecturally consequential decision so far. Batch 2 contains two follow-up questions (Q9, Q10) that nail down the custom format's design intent before we go further. Until those are answered, the writer module is a black box stub.

**Open questions deferred to Batch 2**:
- What is the format's design priority? Smallest size? Fastest mmap-decode? Stable ABI with the RS-NZM-GPCT estimator? Self-describing/versioned?
- Do we still want a parallel "raw SBF passthrough" file as an optional fallback (e.g. for offline debugging with RxTools)?
- Byte order, alignment, framing, version byte, file-level checksum?
- How are SBF time fields (`TOW` ms-of-week + `WNc` week number) represented? Preserved verbatim, converted to nanoseconds-since-Unix-epoch, or both?

**Architectural implication**:
- The writer module has a `Format` interface; v1 ships at least one impl (`CustomBinaryWriter`), with provision for a future `RawSbfWriter` if "double-write" is selected in Q10.
- Lossless capture (CLAUDE.md priority #2) is now defined as "every decoded block reaches the writer without data-field loss", *not* "every byte from the wire is on disk". This must be explicit in the architecture docs.

---

### A5. File rotation policy — Q5 = D (default)

**User answer**: Rotate when the current file reaches **1 GB OR 1 hour**, whichever comes first.

**Architectural implication**:
- The writer holds a single mutable `current_file_` handle plus running byte and start-time counters.
- Rotation check is done on every block-append. Rotation is synchronous (close → rename to final name → open next).
- Rotation must never tear a block — a block is written atomically into the current file even if it crosses the size threshold; the threshold is checked *after* the append.
- File naming: `asterx-YYYYMMDDTHHMMSSZ-<seq>.<ext>` where `<ext>` depends on Q9 outcome.

---

### A6. Connection mode — Q6 = D (default)

**User answer**: Support both **plaintext on TCP 28784** and **TLS on TCP 28783**, selectable via config; default to plaintext in dev, TLS in production.

**Architectural implication**:
- `Transport` interface with two implementations: `PlainTcpTransport` (Boost.Asio sockets) and `TlsTcpTransport` (Boost.Asio SSL stream).
- Both implementations share the same `read/write` virtual API; the IPS data channel (Q7) and the control channel each open a `Transport`.
- TLS uses the system CA store by default; a `ca_bundle_path` config key allows a self-signed receiver certificate (factory-default Septentrio certs are self-signed).

---

### A7. Independent IPS data channel — Q7 = D (default)

**User answer**: Yes — open a **dedicated TCP connection to an IPS server (e.g. `IPS1`)** for the SBF stream, separate from the command/login connection.

**Rationale**: Isolates bulk data from control traffic; command latency and parse-error handling stay clean; reconnect logic per-channel.

**Architectural implication**:
- Two `Transport` instances per session: `ctrl_transport_` (login + commands + replies) and `data_transport_` (SBF stream only, no commands).
- `ctrl_transport_` runs an ASCII line-oriented dialogue (`echo Hello`, `login user pass`, `setSBFOutput Stream1 IPS1 …`, etc.); `data_transport_` is read-only.
- On connect:
  1. Open `ctrl_transport_`, log in, identify receiver, push config.
  2. Configure `IPS1` to listen on a chosen TCP port (e.g. 28785) via `setIPServerSettings`.
  3. Open `data_transport_` to that port, validate first SBF sync byte, hand off to the parser.

---

### A8. IP discovery / addressing — Q8 = D (default)

**User answer**: Support all three discovery mechanisms — **DHCP + mDNS** (zero-conf, `asterx-i3-<serial>.local`), **static IP** (config override), and let the user pick per session.

**Architectural implication**:
- Config has three keys: `connection.host` (string), `connection.discovery_method` (`mdns` | `static` | `auto`), `connection.serial` (optional, for mDNS hostname construction).
- `auto` tries: (1) static if `host` is an IP literal, (2) mDNS if `host` ends in `.local` or `serial` is provided, (3) fail with diagnostic.
- mDNS is provided by Avahi on Ubuntu 20.04 (`libavahi-client-dev`); we use it through a thin wrapper to keep the dependency optional at compile time (`-DASTERX_WITH_MDNS=ON` default).

---

## B. Custom binary format, receiver config, threading (Q9–Q15, ANSWERED)

### B1. Custom binary format priorities — Q9 = D (default)

**User answer**: Priority order =
1. **Timestamp fidelity** (preserve TOW + WNc verbatim; no precision loss),
2. **ABI alignment with downstream estimator** (records map directly to estimator-side structs),
3. **Self-describing / versioned** (file carries a schema descriptor so the decoder is forward-compatible),
4. **Fast mmap-decode** (zero-copy reinterpret on aligned, fixed-size records),
5. **Minimum size** (compression is the lowest priority; v1 ships uncompressed).

**Rationale**: The user is a researcher running batch post-processing through a continuous-time GNSS-INS estimator (RS-NZM-GPCT); cheap re-reads dominate the workload, while disk-space and on-wire bandwidth are not constraints inside the lab. Self-description protects them when the schema evolves across paper revisions.

**Architectural implication**:
- **File header** (mandatory, fixed offset 0): 8-byte magic `ASTRX_BIN`, `uint16 format_version`, `uint8 endianness` (always little-endian for v1, byte enforces it), `uint8 reserved`, `char[32] driver_semver`, `uint32 schema_descriptor_offset`, `uint64 schema_descriptor_length`. Header is itself `alignas(8)` and is exactly 64 bytes.
- **Schema descriptor**: a self-describing block placed at `schema_descriptor_offset` (typically immediately after the header), encoding each record type → fixed size, field name / type / byte-offset list. Encoded as length-prefixed UTF-8 JSON or CBOR (decision to be locked in `05-recording-design.md`). Decoders read this first and need no out-of-band schema knowledge.
- **Record area**: each record is `alignas(8)`, **fixed size per block type** so that `mmap` + `reinterpret_cast<const RecordX*>(ptr + i*sizeof(RecordX))` works without copy. Variable-length SBF blocks (e.g. `MeasEpoch` with N channels) are flattened: a per-record header + a separate "extension area" with a length-prefixed payload. The fixed-size header is mmap-friendly; deep walk requires the extension.
- **Versioning**: `format_version` is bumped on any non-additive change. Additive changes (new optional record type) keep the version and rely on the schema descriptor for discovery.
- **No compression in v1**; the file extension is `.astrx` (custom). A `gzip` outer wrapper is allowed but is not the recorder's responsibility.
- **Deliverable**: a dedicated design doc `docs/architecture/05-recording-design.md` will lock the byte layout, record type IDs, and the schema descriptor encoding. This doc is a prerequisite before the protocol expert is unblocked on the writer.
- **Estimator-side ABI sharing**: the public C++ headers exposing each record struct live in `include/asterx/records/`; the estimator can depend on those headers directly so the same struct definition is used on both sides.

---

### B2. Dual-write raw SBF passthrough — Q10 = D (NO)

**User answer**: Do **not** dual-write a raw SBF file alongside the custom format. The `Format` interface still keeps the extension seam open so a future `RawSbfWriter` can be added without touching the recorder.

**Rationale**: Dual-write halves disk-write headroom and doubles fsync pressure for a fallback path the user does not need in their workflow. Offline RxTools replay is not on the critical path. Keeping the seam open via the `Format` interface costs almost nothing.

**Architectural implication**:
- The writer module exposes `Format` as a pure-virtual interface (`open_session`, `append(BlockRecord)`, `rotate`, `close_session`).
- v1 ships exactly one impl: `CustomBinaryFormat`.
- Configuration key `recording.format = "custom_binary"` is the only legal value in v1, but the field is present (and validated) so adding `"raw_sbf"` later is a one-line config change.
- The recorder is *aware of* a `std::vector<std::unique_ptr<Format>>` so multi-sink fan-out is structurally possible; v1 just builds a vector of length 1.

---

### B3. Timestamp representation — Q11 = D (default)

**User answer**: Preserve the GNSS time triple verbatim: `TOW` (uint32 ms-of-week) + `WNc` (uint16 GPS week number) **plus** a derived `unix_ns` (int64 nanoseconds since Unix epoch). A boolean flag `unix_ns_is_authoritative` is set per-record; default value is **false**, meaning TOW/WNc are the source of truth and `unix_ns` is a *convenience* computed by the driver at write time.

**Rationale**: CLAUDE.md priority #1 is timestamp fidelity. Preserving TOW/WNc means the post-processing pipeline can recompute `unix_ns` with the user's preferred leap-second table at any future date. Carrying a precomputed `unix_ns` makes "quick look" tools (Pandas / matplotlib / spreadsheets) trivial.

**Architectural implication**:
- The shared `BlockHeader` (common prefix of every record) carries three timestamp fields: `tow_ms : uint32`, `wnc : uint16`, `unix_ns : int64`. Plus one bit-flag in a `uint8 flags` byte: `bit0 = unix_ns_is_authoritative`. (Reserved bits in the flag byte will be used for "tow_invalid", "leap_seconds_stale", etc.)
- The driver ships an **embedded GPS leap-second table** current as of **2026-05-16** (most recent leap-second introduction was 2017-01-01, so the current value is `18 s`). The table is a `constexpr` lookup committed in `include/asterx/time/leap_seconds.hpp`.
- A **build-time check** stamps the leap-second table's "last verified" date into the binary. If the recorded `unix_ns` would land more than 365 days past the table's last-verified date, the driver sets `flags.leap_seconds_stale = 1` on every record (still writes `unix_ns`, but consumers are warned).
- Records carrying `TOW = 0xFFFFFFFF` (SBF "do-not-use") are written with `flags.tow_invalid = 1` and `unix_ns` derived from the host wall clock with `unix_ns_is_authoritative = 0`.
- The receiver is configured to deliver TOW with the receiver's best clock estimate; the driver does *not* attempt PPS / NTP cross-checks in v1 (that's a future enhancement).

---

### B4. Receiver-side SBF stream allocation — Q12 = D (default)

**User answer**: Configure **4 SBF output streams**, all on the **single shared IPS1 data port**:

| Stream  | Blocks                                      | Rate     |
|---------|---------------------------------------------|----------|
| Stream1 | `ExtSensorMeas`                             | `msec5`  |
| Stream2 | `MeasEpoch`, `MeasExtra`                    | `msec100`|
| Stream3 | `AuxAntPositions`, `AttEuler`, `AttCovEuler`| `msec100`|
| Stream4 | `ReceiverStatus`, `QualityInd`, `ReceiverTime` | `sec1`   |

**Rationale**: Up to 10 SBF streams exist on the receiver; using 4 leaves headroom and segments blocks by rate-class so a future rate change touches exactly one `setSBFOutput` call. Sharing one IPS port keeps the data-side socket count at 1 (simplifies parser & reconnect logic).

**Architectural implication**:
- The driver's connect sequence sends, in order: `setDataInOut`, `setIPServerSettings IPS1 …`, then four `setSBFOutput Stream{1..4} IPS1 …` commands.
- The exact `setSBFOutput` argument strings (block-list spelling per Firmware v1.5.2) will be locked in `docs/architecture/04-receiver-config.md`.
- Stream-to-rate mapping is **defaults**; per A3 (Q3), the actual rates come from config and can override these mappings. Stream membership (which blocks go on which stream) is also overridable, with validation that no block appears on two streams.
- All 4 streams demux into the same TCP socket; the SBF parser is unaware of stream IDs (SBF frames carry block number, not stream number).

---

### B5. IMU orientation — Q13 = D (default)

**User answer**: Expose IMU orientation as **three driver-config angles** `imu.orientation.theta_x_deg`, `imu.orientation.theta_y_deg`, `imu.orientation.theta_z_deg` (Tait-Bryan, rotation from receiver body to IMU sensor body, per Septentrio convention). Default is the receiver factory `SensorDefault` orientation `(0, 0, 0)`.

**Rationale**: Mounting-dependent. Keeping it in driver config (not on-receiver `boot`) lets the user swap rigs without flashing the receiver.

**Architectural implication**:
- The connect sequence sends `setImuOrientation manual <θX> <θY> <θZ>` (or `setImuOrientation SensorDefault` when all three are zero AND `imu.orientation.use_sensor_default = true`).
- Config validation: each angle must be in `[-180.0, 180.0]` degrees.
- The numerical values are also written into the recording's schema descriptor so post-processing can recover the orientation that was active at capture time.

---

### B6. Receiver-config persistence — Q14 = D (default)

**User answer**: Driver applies all configuration to the receiver's **`current`** configuration only. **Never** copy to `boot`. The receiver returns to its previously-flashed boot config on power cycle.

**Rationale**: Avoids contaminating the receiver's persistent state. Each session re-applies the config it needs; the receiver is treated as stateless from the driver's point of view.

**Architectural implication**:
- The driver never issues `exeCopyConfigFile, Current, Boot`. Forbidden in code; a unit test asserts no such string is emitted.
- A `boot` mode (push to flash) is intentionally not exposed in v1. If a future user needs it, it's a deliberate feature add, not a flag.
- Documentation will explicitly state: "every session begins by re-pushing the full command sequence; rebooting the receiver returns it to factory/last-flashed state."

---

### B7. Threading model — Q15 = D (default)

**User answer**: Three pinned worker threads plus a callback-dispatch thread pool:

| Thread                | Owner / role                                                                  | Boost.Asio? |
|-----------------------|-------------------------------------------------------------------------------|-------------|
| `io_thread`           | One `boost::asio::io_context`; runs **both** TCP sockets (ctrl + data) via async ops | Yes |
| `parser_thread`       | Consumes raw bytes off an SPSC ring, runs the SBF byte-machine, emits `BlockRecord`s | No |
| `writer_thread`       | Consumes `BlockRecord`s off an SPSC ring, runs the active `Format` impl, flushes to disk | No |
| `subscriber_pool` (N) | Best-effort fan-out to real-time subscribers; default N=2, configurable        | No |

Inter-thread comms use `boost::lockfree::spsc_queue` between the three primary stages; the subscriber pool consumes the same outbound queue via an MPMC overlay (or a per-subscriber SPSC + a fan-out thread; final choice locked in `02-threading-model.md`).

**Rationale**: Decouples wire I/O, parsing, and disk I/O so disk latency cannot back-pressure the IMU stream. Single-producer-single-consumer queues avoid mutex contention on the hot path. Subscriber pool is isolated so a slow downstream consumer can never block recording (per A1).

**Architectural implication**:
- A dedicated design doc `docs/architecture/02-threading-model.md` will spell out:
  - Thread CPU affinity strategy (pinning to dedicated cores via `pthread_setaffinity_np`, configurable on / off; the user's lab boxes are NUMA-quiet so default is off).
  - Real-time priority (use `SCHED_FIFO` for `io_thread` and `parser_thread` if running as root, fall back to `SCHED_OTHER` with `nice -10`; configurable).
  - Queue depths (default: io→parser `64 KiB` raw buffer, parser→writer `8192` `BlockRecord` slots, parser→subscriber-fanout `4096`).
  - Back-pressure policy: writer queue full = increment `recorder_drops_total` counter and **block the parser** for up to `max_block_ms` (default 10 ms); after that, abort the session with `ESTALE`. Subscriber queue full = drop frame for *that* subscriber with a per-subscriber counter; never blocks.
  - Shutdown ordering: stop io_thread first, drain parser, drain writer, close file, then join.
- The `boost::lockfree::spsc_queue<T, capacity>` requires `T` to be trivially-destructible. `BlockRecord` is therefore a POD-ish struct with optional heap payload stored as `std::byte*` + `size_t` + `std::pmr::memory_resource*` (a small-object pool allocator); ownership is transferred by pointer, not RAII, on the hot path.
- Sanitizer support: `ASTERX_ENABLE_TSAN` adds a CMake option that swaps lockfree queues for mutex-protected ones (TSan can't track lockfree primitives reliably).

---

## C. Errors, logging, config, CLI, credentials, reconnect (Q16–Q22, ANSWERED)

### C1. Error-handling strategy — Q16 = D (default)

**User answer**: **Hybrid error propagation**. The hot path (transport → parser → writer) returns `tl::expected<T, asterx::Error>` (no exceptions on per-block code paths). The public facade (`asterx::Driver` lifecycle methods: `start()`, `stop()`, `connect()`) throws `asterx::Exception` for irrecoverable startup-time conditions. Destructors and registered callbacks never throw — any exception escaping a callback is caught at the dispatcher boundary, logged, and the subscriber is marked degraded.

**Rationale**: Hot-path exceptions are expensive (unwinding on every CRC failure would be visible at 200 Hz IMU rates) and obscure data-flow logic. `tl::expected` lets the SBF byte-machine carry error context (frame offset, CRC mismatch, length mismatch) without per-frame allocations. Reserving exceptions for the facade keeps the call-site ergonomic for the human operator (`try { driver.start(); } catch (const asterx::Exception& e) { ... }`).

**Architectural implication**:
- Add `include/asterx/error.hpp` defining `enum class ErrorCode { ... }` (e.g. `CrcMismatch`, `LengthMismatch`, `UnknownBlock`, `TransportClosed`, `Timeout`, `LoginFailed`, `ConfigInvalid`, `ReceiverUnreachable`, …) and `struct Error { ErrorCode code; std::string context; }` along with `asterx::Exception : std::runtime_error` (carrying an `Error` payload).
- Adopt `tl::expected<T, asterx::Error>` (header-only library, FetchContent-pinned). C++23 `std::expected` would be preferable but is unavailable on Ubuntu 20.04 stock GCC 9.4 — the eventual upgrade path is a `using` alias swap.
- Destructors are marked `noexcept` (default) and use `try/catch(...)` around any potentially-throwing cleanup (e.g. `close_file()` log lines).
- Callback dispatch wraps each user callback in a `try/catch (...)` that logs `"subscriber id=%u threw: %s"` and degrades the subscriber (3 strikes → unsubscribe).
- A `noexcept`-correctness lint (clang-tidy `bugprone-exception-escape`) is enabled in CI.

---

### C2. Logging — Q17 = B (**deviation from default**)

**User answer**: **spdlog with a rotating file sink ONLY** — `logs/asterx-YYYYMMDD.log`, 5 files × 100 MiB rotation. **No console / stderr sink under normal operation.**

**Rationale (user-stated)**: The driver runs unattended in lab sessions; the operator does not watch the console. All useful diagnostics live in the daily log files alongside the recordings.

**Architectural implication**:
- Logger initialization builds **exactly one** `spdlog::sinks::rotating_file_sink_mt` (path from config, default `./logs/asterx-YYYYMMDD.log`, 100 MiB × 5). **No `dist_sink_mt`, no `stdout_color_sink_mt`** in the steady-state logger setup.
- **EXCEPTION — startup-phase fatal errors**: from process start until the moment the driver reaches the **"data flow established"** state (i.e. control connection up, login succeeded, IPS data port open, first SBF sync byte observed) OR **until the first 3 seconds elapse**, whichever comes first, **fatal errors MUST also emit a single concise line to `stderr`**. This is a deliberate narrowing of "no console sink": without it, a researcher launching `./asterx_driver --config x.toml` and hitting a typo / unreachable receiver / wrong password sees absolutely nothing in the terminal. The stderr path uses a temporary `stderr_sink` attached only during the startup window, then detached after data-flow-established.
- Log level configurable via TOML `logging.level = "info"` (trace/debug/info/warn/err/critical) AND via `--log-level` CLI flag (CLI overrides TOML).
- Log line format: `[YYYY-MM-DD HH:MM:SS.fff] [thread_name] [level] message` (spdlog pattern `[%Y-%m-%d %H:%M:%S.%e] [%n] [%^%l%$] %v`).
- Async logging via `spdlog::async_logger` with a 8192-slot queue and overflow policy `overrun_oldest` (the recording is the source of truth, not the log; we accept dropped log lines under saturation rather than blocking the hot path).
- **Never log credentials**: a dedicated `redact()` helper masks any field whose key matches `/pass|token|secret|key/i` to `[REDACTED]`. Wrapped in a unit test.
- Log-file naming uses date-only (not session-id) so multiple driver runs on the same day append; rotation handles size. Documentation will state this explicitly so the user expects daily files, not per-session files.

---

### C3. Bad-frame handling — Q18 = D (default)

**User answer**: **Drop / count / log / watchdog** — silently drop frames that fail CRC or length checks, increment a counter, log at `warn` once per second, and trigger a watchdog if the bad-frame rate crosses a threshold.

**Rationale**: At 200 Hz IMU + multi-Hz GNSS, transient bit-flips on a wire or a partial socket read happen; aborting the session on one bad frame is over-reaction. But a *sustained* bad-frame stream signals a real problem (wrong endianness, wrong port, line noise, receiver malfunction) and must surface fast.

**Architectural implication**:
- Parser maintains `std::atomic<uint64_t>` counters: `frames_total`, `frames_crc_fail`, `frames_length_fail`, `frames_unknown_block`, `bytes_resynced`.
- Counters are exposed via the public API (`Driver::metrics() -> Metrics`) for downstream monitoring and emitted into the rotating log every 1 s at `info` level (configurable interval via `logging.metrics_interval_ms`).
- **Watchdog**: a sliding 10 s window; if `frames_crc_fail / frames_total > 0.05` (5 % bad-frame rate) AND `frames_total > 100` within the window, the parser raises `ErrorCode::ParserDegraded`. The driver facade decides whether to abort or reconnect (see C7 / Q22).
- On the wire, partial-frame resync uses the SBF sync sequence `0x24 0x40` (`$@`); the parser scans forward byte-by-byte and counts `bytes_resynced` for visibility. No data-loss assertions are made about resync bytes — they are by definition unparseable.
- Counters reset on each successful reconnect.

---

### C4. Configuration file — Q19 (default adjusted per Q20=A)

**User answer (Q19)**: **TOML via `toml++`**, schema documented in `docs/architecture/06-config-schema.md`, an example file at `examples/asterx.toml`.

**ADJUSTMENT due to Q20=A (minimal CLI)**: The original default also included a `--validate-config <path>` CLI subcommand. **Per Q20=A this subcommand is dropped.** Schema validation now runs **unconditionally at driver startup**: if the TOML fails to parse, fails schema validation (unknown key, wrong type, out-of-range value, missing required field), or refers to a file path that cannot be opened, the driver prints a multi-line error to stderr (within the startup-phase stderr window — see C2), logs the same to the log file, and exits with code **2** (config error; distinct from code 1 = runtime error).

**Rationale**: TOML is human-readable, well-typed (vs YAML's tag soup), and `toml++` is a single-header library with a clean Boost-licensed C++17 API. Forcing validation at startup (rather than lazily) means the operator catches typos before the receiver dialog even begins.

**Architectural implication**:
- Add `toml++` via `FetchContent` (pinned commit). Header-only, no runtime dep.
- A `Config` struct mirrors the schema; each field documents its TOML path, type, default, and validation rule. Field validators live next to the field (lambda / `std::function<Status()>`); a top-level `Config::validate()` calls every field validator and aggregates errors before returning (so one error message lists ALL problems, not just the first).
- The schema doc `06-config-schema.md` is the human-readable contract; the `Config` struct is the machine-readable one. A doctest in CI parses every example in the doc and asserts it validates.
- **No `--validate-config` subcommand** in v1. Reproducing "validate without running" can be done by pointing `--config` at a file and capturing exit code 2; if a non-destructive dry-run is wanted later it would be added as a config field (`run_mode = "validate" | "record"`), not as a CLI subcommand.
- **Defaults policy**: every config key has a documented default; the minimum legal TOML is an empty file (which yields all defaults). The operator can override progressively.

---

### C5. Command-line interface — Q20 = A (**deviation from default**)

**User answer**: **Minimal CLI: only `--config <path>`.** No subcommands. The single binary `asterx_driver` performs exactly one action — start the recording session described by the config file.

**Rationale (user-stated)**: The user is the sole operator; mental model is "config file is the program, CLI just points at it". Subcommands add API surface and tests for no operational benefit.

**Architectural implication**:
- Single binary: `asterx_driver`.
- Accepted flags (and *only* these):
  - `--config <path>` — required. Path to TOML config.
  - `--log-level <trace|debug|info|warn|err|critical>` — optional. Overrides `logging.level` from TOML. Justified because operators commonly want a one-shot log bump without editing the config.
  - `--help` / `-h` — prints flag list AND a pointer to the config-schema doc (`See docs/architecture/06-config-schema.md or examples/asterx.toml for config options.`).
  - `--version` / `-V` — prints driver semver + git SHA + build timestamp.
- **No subcommands** (`record`, `probe`, `replay` are NOT exposed in v1).
- A future `probe` mode (just-verify-connectivity-and-identify-receiver) is reproducible by setting a future config field `run_mode = "probe"` — **but `probe` is explicitly NOT implemented in v1**. v1 has exactly one run mode: record.
- Argument parsing uses `CLI11` (single-header, FetchContent). Even though the flag set is tiny, CLI11 gives consistent `--help` formatting, error reporting, and saves us hand-rolling parser code that breaks on edge cases (e.g. `=` vs space, short flag bundling).
- Exit codes:
  - `0` = clean shutdown (e.g. `SIGINT` received, draining complete).
  - `1` = runtime error after startup (lost connection, watchdog tripped, disk full).
  - `2` = config error (parse fail, validation fail, file not found at startup).
  - `3` = CLI usage error (unknown flag, missing required flag).
  - `64` = reserved for `--help` / `--version` (CLI11 default is 0; we accept that).

---

### C6. Credential handling — Q21 = D (default)

**User answer**: **Layered credential resolution**. In order of priority (later layers override earlier):
1. `ASTERX_USERNAME` / `ASTERX_PASSWORD` environment variables.
2. A sealed (chmod-600) file at the path given by `connection.credentials_file` in TOML, format `username=...\npassword=...`. The driver verifies file mode is `0600` (or `0400`) before reading; refuses otherwise.
3. Plaintext `connection.username` / `connection.password` in the TOML config, **with a `warn`-level log line** "credentials in plaintext TOML — consider env vars or a sealed credentials_file".
4. Interactive prompt on stderr (only if `connection.allow_interactive_login = true` AND the process has a TTY).

If after all layers no credentials are present, the driver exits with code 2 and the error message lists all four layers it tried.

**Rationale**: Researcher-friendly defaults (TOML for dev) without forcing bad habits (warn on plaintext); production paths (env vars / sealed file) are first-class.

**Architectural implication**:
- A `CredentialResolver` class in `src/credentials/` walks the layers and returns `tl::expected<Credentials, Error>`.
- File-mode check uses `stat(2)` + `S_IRGRP | S_IROTH | S_IWGRP | S_IWOTH`; refusal is a hard error with code 2.
- Interactive prompt uses `termios` to disable echo on password input.
- Credentials are stored in `std::string` wrapped in a small RAII `SecureString` type that zeros memory on destruction. We do NOT mlock — the threat model is "don't print to logs / don't dump in core file"; full anti-cold-boot defense is out of scope.
- A unit test asserts that `Credentials::operator<<` and `spdlog` formatter both render `[REDACTED]`.

---

### C7. Reconnect policy — Q22 = D (default)

**User answer**: **Probe-then-decide reconnect** with a 5 s data-silence watchdog.

**Rationale**: A blind exponential-backoff reconnect can mask a misconfiguration ("password changed at receiver and now nobody can log in") and waste hours of recording time. A probe-then-decide reconnect distinguishes transient failures (link flap, wifi blip, transient TLS handshake error) from semantic failures (auth fail, port not listening) and only retries when retry is meaningful.

**Architectural implication**:
- **Data-silence watchdog**: `data_transport_` arms a 5 s timer on every successful SBF frame; if the timer expires the data channel is declared dead. The 5 s default is configurable via `connection.data_silence_timeout_ms` (allowed range 1000–60000 ms).
- **Probe phase**: on disconnect, the driver opens a **fresh control connection** and runs a 4-step probe: (1) TCP connect within 2 s, (2) banner read within 1 s, (3) login within 2 s, (4) `getReceiverCapabilities` query within 2 s. Each step has its own timeout; failure of any step is classified.
- **Classification table**:
  - **Transient** (TCP connect timeout, TCP RST, banner timeout, TLS handshake retryable error): retry with exponential backoff `1s → 2s → 4s → 8s → 16s → 30s`, then 30 s flat. Infinite retries.
  - **Semantic** (login refused, "user not authorized", receiver capability mismatch, schema-incompatible firmware): **do NOT retry**. Log critical, exit with code 1. The operator must intervene.
  - **Ambiguous** (3 consecutive transient failures with identical error message): elevate to semantic; treat as "stuck", exit code 1.
- During the probe / backoff window, the writer stays open and the current file is flushed-but-not-rotated. On successful reconnect, the writer logs a "gap marker" record (a synthetic block recording wall-time gap + frame counter delta) so post-processing can see the discontinuity.
- Counters: `reconnect_attempts_total`, `reconnect_successes_total`, `reconnect_classified_semantic_total`, `bytes_received_total`, `frames_received_total`, `data_silence_events_total`, all in `Driver::metrics()`.
- A future enhancement (NOT v1) could add receiver-side `IPS1` re-arming if the receiver itself dropped the listener; v1 just re-opens the data socket and relies on the receiver to still be listening after the control channel is restored.

---

## D. Testing, build, delivery (Q23–Q30, ANSWERED)

### D1. Test strategy — Q23 = D (default)

**User answer**: A **three-layer test pyramid**:
1. **Unit tests** — fast, deterministic, in-process. Cover the SBF byte-machine, CRC implementation, every block parser, frame resync, the `Format` writer impl (round-tripping records), the config schema, the credential resolver, the leap-second table, and the watchdog state machine.
2. **Integration tests** — drive the `Transport` layer through a `socat`-spawned TCP fixture that replays canned SBF byte streams. Validates the IO thread, the IO→parser ring buffer, and end-to-end timing under realistic socket-read chunking (not just clean `recv()` of whole frames).
3. **Replay round-trip tests** — given a real captured SBF stream (TCP dump or `.sbf` file), feed bytes through the full pipeline, recording to `.astrx`, then read back the `.astrx` and assert every record reproduces the SBF-side field values modulo documented transformations (TOW preserved verbatim, `unix_ns` derived).

**Deferred to a later milestone (NOT v1)**: protocol-level fuzzing (libFuzzer/AFL on the SBF parser), property-based testing (Rapidcheck), TSan on the live integration tests under real hardware.

**Rationale**: Three layers cover correctness (unit) + plumbing (integration) + end-to-end semantics (replay) with minimal overlap. Fuzzing is the natural next milestone once the protocol is shaped to use it; doing it pre-stabilization wastes corpus work.

**Architectural implication**:
- Each `src/` module owns a sibling test target: `src/sbf/` ↔ `tests/unit/sbf/`.
- A `tests/fixtures/` directory holds canned SBF byte streams; `tests/replay/` holds whole-session captures.
- `socat` is a runtime test dependency (Ubuntu `apt install socat`); declared in `docs/USAGE.md` and CI workflow.
- CI runs all three layers on every PR. Replay tests are tagged so they can be opt-out for `git bisect` runs where a fixture isn't yet checked in.
- `ctest --output-on-failure -L unit` / `-L integration` / `-L replay` provide selective execution.

---

### D2. Test framework — Q24 = A (default)

**User answer**: **GoogleTest + GMock**, vendored via `FetchContent` at tag `v1.14.0` (pinned).

**Rationale**: GTest is the de facto C++ unit-test framework; integrates cleanly with CTest; GMock is needed to mock the `Transport` interface (so unit tests don't open real sockets). v1.14.0 is the current stable release on Ubuntu 20.04 toolchains and supports C++17 cleanly.

**Architectural implication**:
- `cmake/FetchGoogleTest.cmake` declares `FetchContent_Declare(googletest GIT_REPOSITORY https://github.com/google/googletest.git GIT_TAG v1.14.0)`.
- `gtest::gtest_main` is the default linker target for unit tests; `gmock::gmock` for integration tests that need a mock `Transport`.
- A small CMake macro `asterx_add_test(<name> SRCS ... LIBS ... LABELS ...)` wraps `add_executable` + `target_link_libraries` + `gtest_discover_tests` + sanitizer flag propagation, keeping per-test CMakeLists.txt one-liner.
- No `apt`-installed GTest is consulted (Ubuntu 20.04 ships 1.10 which lacks `MOCK_METHOD` ergonomics); the `FetchContent` build is always used.

---

### D3. Third-party library inventory — Q25 = A (default)

**User answer**: The full third-party stack is **locked**. Use this table as the authoritative dependency list for Phase 2.

| Library              | Source              | Pinned version / commit | Purpose                                          |
|----------------------|---------------------|-------------------------|--------------------------------------------------|
| Boost.Asio           | `apt` system        | 1.71 (Ubuntu 20.04 system) | Async sockets, SSL, lockfree                  |
| Boost.System         | `apt` system        | 1.71                    | Asio dep                                          |
| Boost.LockFree       | `apt` system        | 1.71                    | `spsc_queue` between threads                      |
| OpenSSL              | `apt` system        | 1.1.1f (Ubuntu 20.04 system) | TLS via Boost.Asio SSL stream                |
| spdlog               | `FetchContent`      | v1.13.0                 | Logging                                           |
| fmt                  | spdlog-bundled      | bundled (v9.x via spdlog) | spdlog dep; we don't fetch it standalone        |
| toml++               | `FetchContent`      | v3.4.0                  | TOML config parsing                               |
| CLI11                | `FetchContent`      | v2.4.2                  | CLI flag parsing                                  |
| tl::expected         | `FetchContent`      | v1.1.0                  | `tl::expected<T, asterx::Error>` hot-path return |
| Avahi (libavahi-client) | `apt` system     | 0.7 (Ubuntu 20.04 system) | mDNS discovery (optional, `-DASTERX_WITH_MDNS=ON`) |
| GoogleTest + GMock   | `FetchContent`      | v1.14.0                 | Unit / integration tests                          |
| lcov                 | `apt` system        | 1.14 (Ubuntu 20.04 system) | Coverage reporting (CI only)                   |
| socat                | `apt` runtime       | 1.7.3                   | Integration-test TCP fixture                      |

**Rationale**: System packages where Ubuntu 20.04 already ships an acceptable version (avoids re-building Boost / OpenSSL / Avahi in every CI run, ~5 min saved). `FetchContent` for header-only / template-heavy libraries where the system version is too old or absent. Every `FetchContent` line carries a `GIT_TAG <tag>` AND a `GIT_SHA <hash>` comment so a Renovate / Dependabot bump is reviewable.

**Architectural implication**:
- `cmake/FetchDependencies.cmake` is the single entry point for all `FetchContent_Declare` calls; called once from top-level `CMakeLists.txt`.
- The `find_package(Boost 1.71 REQUIRED COMPONENTS system)` call gates the build with a clear error message if the system Boost is too old.
- `find_package(OpenSSL REQUIRED)` and `find_package(Avahi)` (optional) round out the system-dep block.
- The full dependency tree (transitive included) will be visualized in `docs/architecture/05-build-and-deps.md` as a Mermaid graph.
- Reproducibility: every `FetchContent_Declare` uses both `GIT_TAG` AND `GIT_PROGRESS TRUE`; bare `master` is forbidden.

---

### D4. CMake project structure — Q26 = A (default)

**User answer**: **Multi-target monorepo** CMake layout:

```
asterx_driver/
├── CMakeLists.txt                  # top-level: project(), options, FetchDependencies, add_subdirectory
├── cmake/                          # helper modules
│   ├── FetchDependencies.cmake     # all FetchContent_Declare in one place
│   ├── CompilerWarnings.cmake      # -Wall -Wextra -Wpedantic -Werror profile
│   ├── Sanitizers.cmake            # ASTERX_ENABLE_{ASAN,UBSAN,TSAN}
│   ├── AsterXAddTest.cmake         # asterx_add_test() macro
│   └── AsterXInstall.cmake         # install() / export targets
├── include/asterx/                 # PUBLIC headers — installed
│   ├── driver.hpp                  # the facade
│   ├── error.hpp                   # Error + Exception + ErrorCode
│   ├── config.hpp                  # Config struct + validators
│   ├── metrics.hpp                 # Metrics struct (counters)
│   ├── records/                    # record-type headers (shared with estimator)
│   └── time/leap_seconds.hpp       # leap-second table
├── src/                            # PRIVATE implementation
│   ├── _core/                      # CMake target: asterx_core (STATIC lib)
│   │   ├── transport/              # owned by ethernet-protocol-expert
│   │   ├── protocol/               # owned by ethernet-protocol-expert
│   │   ├── sbf/                    # owned by ethernet-protocol-expert
│   │   ├── recording/              # writer + Format impls
│   │   ├── credentials/            # CredentialResolver
│   │   ├── config_parser/          # TOML parser + validator
│   │   ├── time/                   # leap-second math
│   │   └── driver.cpp              # facade impl (cpp-architect-owned)
│   └── CMakeLists.txt              # builds asterx_core STATIC
├── apps/
│   ├── asterx_driver/              # the single executable
│   │   ├── main.cpp                # arg parse → Config → Driver → run
│   │   └── CMakeLists.txt
│   └── CMakeLists.txt
├── tests/
│   ├── unit/                       # ctest -L unit
│   ├── integration/                # ctest -L integration (socat fixtures)
│   ├── replay/                     # ctest -L replay (canned .sbf)
│   ├── fixtures/                   # binary SBF streams
│   └── CMakeLists.txt
├── examples/
│   ├── asterx.toml                 # the canonical sample config
│   ├── subscriber_demo.cpp         # uses the public Subscriber API
│   └── CMakeLists.txt
├── third_party/                    # vendored sources, if any (currently empty; everything via FetchContent)
└── docs/                           # architecture docs + Doxygen output (not built by default)
```

**Rationale**: A clean separation between the library (`asterx_core` STATIC) and the executable (`asterx_driver`) means downstream consumers (the user's RS-NZM-GPCT factor-graph estimator) can link directly against the library without dragging in `main.cpp` / CLI11. Per-folder `CMakeLists.txt` keeps each target's build self-contained.

**Architectural implication**:
- The CMake target graph: `asterx_core` (STATIC) ← `asterx_driver` (executable), `asterx_test_*`, `asterx_example_*`.
- `asterx_core` is the only target that propagates `target_link_libraries(... PUBLIC ...)` for headers + `PRIVATE` for internal libs. All `target_*` modern CMake — no `include_directories`, no `link_libraries`.
- `install(TARGETS asterx_core EXPORT AsterXTargets ...)` plus an `AsterXConfig.cmake` lets a downstream consumer do `find_package(AsterX REQUIRED)` and `target_link_libraries(my_estimator PRIVATE AsterX::asterx_core)`.
- The `_core` underscore prefix in `src/_core/` is purely a sort-order convention so it appears at the top of file listings — semantically it's the canonical library target.
- A `cpack` configuration is **out of scope for v1** (research deployment is git-clone + cmake).

---

### D5. Sanitizers — Q27 = A (default)

**User answer**: Three CMake options, **off by default**, mutually exclusive at configure time:

| Option                    | Build flags added                                          | CI job          |
|---------------------------|------------------------------------------------------------|-----------------|
| `ASTERX_ENABLE_ASAN=ON`   | `-fsanitize=address -fno-omit-frame-pointer -O1`           | `ci-asan`       |
| `ASTERX_ENABLE_UBSAN=ON`  | `-fsanitize=undefined -fno-sanitize-recover=undefined`     | `ci-ubsan` (typically combined with ASAN) |
| `ASTERX_ENABLE_TSAN=ON`   | `-fsanitize=thread -O1`                                    | Not in default CI; run locally on demand |

**TSan ⇄ lock-free queue conflict**: TSan cannot reliably model `boost::lockfree::spsc_queue`'s atomic operations and emits false positives. When `ASTERX_ENABLE_TSAN=ON`, the CMake build **automatically swaps** the lockfree queues for a mutex-protected `std::queue<T> + std::mutex + std::condition_variable` adapter that exposes the same `push() / pop()` API. The swap is implemented via a CMake-controlled `#define ASTERX_QUEUE_MUTEX_FALLBACK` propagated into the `asterx_core` target as a compile definition.

**Rationale**: ASAN catches most heap/use-after-free bugs at low overhead (~2x). UBSAN catches signed overflow, null-deref, alignment violations, etc. — critical for a binary-format writer. TSan is a separate-runtime tool used during development of the threading model and the back-pressure path; running it in default CI alongside `boost::lockfree` produces noise. The lockfree→mutex swap is a documented degradation mode, not a bug.

**Architectural implication**:
- `cmake/Sanitizers.cmake` exposes the three options + asserts mutual exclusivity (only one of ASAN/UBSAN/TSAN can be `ON` simultaneously; combining ASAN+UBSAN is allowed and is the default CI build).
- CI runs a dedicated `ci-asan-ubsan` job using `cmake -DASTERX_ENABLE_ASAN=ON -DASTERX_ENABLE_UBSAN=ON -DCMAKE_BUILD_TYPE=Debug ..`.
- The lockfree→mutex adapter (`src/_core/util/queue.hpp`) is a templated `Queue<T>` that conditionally aliases to `boost::lockfree::spsc_queue<T>` or to the mutex-backed variant. Both pass identical unit tests; only the mutex variant is used under TSan.
- A `docs/architecture/02-threading-model.md` section documents the TSan caveat so a future maintainer doesn't try to "fix" the apparent inconsistency.

---

### D6. Continuous integration — Q28 = A (default)

**User answer**: **GitHub Actions** with a build-matrix covering Ubuntu 20.04 + 22.04 across four compilers, plus a dedicated sanitizer job.

**Matrix** (`.github/workflows/ci.yml`):

| OS              | Compiler  | Build type | Sanitizers   | Notes                                  |
|-----------------|-----------|------------|--------------|----------------------------------------|
| ubuntu-20.04    | gcc-9     | Debug      | none         | Stock Ubuntu 20.04 toolchain — primary target |
| ubuntu-20.04    | gcc-11    | Release    | none         | From `ubuntu-toolchain-r/test` PPA      |
| ubuntu-20.04    | clang-10  | Debug      | none         | Stock 20.04 clang                       |
| ubuntu-20.04    | clang-14  | RelWithDebInfo | none     | LLVM apt repo                           |
| ubuntu-22.04    | gcc-11    | Debug      | none         | Forward-compat check                    |
| ubuntu-22.04    | gcc-12    | Release    | none         |                                         |
| ubuntu-22.04    | clang-14  | Debug      | none         |                                         |
| ubuntu-22.04    | clang-15  | RelWithDebInfo | none     |                                         |
| ubuntu-22.04    | gcc-12    | Debug      | ASAN+UBSAN   | Dedicated sanitizer job (the 9th cell)  |

**That's 8 build-matrix jobs + 1 sanitizer job = 9 CI cells per PR.**

**Job actions per cell**:
1. `apt install` system deps (Boost, OpenSSL, Avahi, socat).
2. `cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=$BT $SANITIZER_FLAGS`.
3. `cmake --build build --parallel`.
4. `ctest --test-dir build -L unit --output-on-failure` (unit tests).
5. `ctest --test-dir build -L integration --output-on-failure` (socat-backed).
6. `ctest --test-dir build -L replay --output-on-failure` (canned SBF replay).
7. On `ubuntu-20.04 + gcc-9 + Debug`: also run `lcov` and upload coverage to a `coverage/` artifact.

**Rationale**: Two-OS coverage catches forward-compat issues early. Four compilers per OS (two GCC, two Clang) catches diagnostic / warning differences. A dedicated sanitizer job avoids slowing down the entire matrix. Ubuntu 20.04 + gcc-9 is the primary target so it runs in Debug for full assertion coverage.

**Architectural implication**:
- `.github/workflows/ci.yml` uses GitHub's `matrix.strategy.fail-fast: false` so a single compiler flake doesn't kill all 9 jobs.
- The integration tests require `socat`; the workflow installs it explicitly.
- Coverage upload uses `lcov` → `genhtml` → upload-artifact. We do NOT use Codecov.io in v1 (no SaaS dependency).
- A separate `clang-tidy` job (not in the matrix; a single Ubuntu 22.04 + clang-15 cell) runs `run-clang-tidy` over `compile_commands.json` and uploads diagnostics as a CI artifact.
- A `clang-format-check` job runs `clang-format --dry-run --Werror` over `src/`, `include/`, `apps/`, `tests/`, `examples/`.
- A nightly cron job (separate workflow) runs the TSan build manually on Ubuntu 22.04 + gcc-12; failure does NOT block PRs (since TSan + lockfree-swap is a documented mode, not a steady-state CI commitment).

---

### D7. Documentation — Q29 = A (default)

**User answer**: **Four-tier documentation**:

1. **`README.md`** — human-facing landing page. What the driver does in 2 sentences; how to build (Docker recipe + bare-metal recipe); how to run a 60-second smoke test; pointer to `docs/USAGE.md` for operations and `docs/architecture/` for design.
2. **`docs/architecture/00–06.md`** — the architect's design docs. `00-requirements.md` (this file) is the contract; `01-component-diagram.md` through `06-config-schema.md` flesh out the design (see Phase 2 plan in the agent system prompt).
3. **`docs/USAGE.md`** — operator's runbook. How to write a config file; what each TOML key does (cross-linked to `06-config-schema.md`); how to interpret log lines; how to read `.astrx` recordings; recovery procedures (disk full, lost connection, wrong credentials).
4. **Doxygen on public headers only** — `include/asterx/*.hpp` carries `///` Doxygen comments; `doxygen Doxyfile` generates `build/doc/html/` (not built by default; opt-in via `cmake --build build --target docs`). Private implementation headers (in `src/`) have only `//` comments.

**Rationale**: Researchers don't read API docs cover-to-cover; they read READMEs to get a build, USAGE to operate, and architecture docs when they want to understand. Doxygen on public headers means the user's downstream consumer code can hover-doc in their IDE without us writing prose for internals.

**Architectural implication**:
- `Doxyfile` template lives at `docs/Doxyfile.in`; configured by CMake to point at `include/asterx/`.
- `docs/USAGE.md` is hand-written, but its config-key reference section is auto-generated from a comment block in `include/asterx/config.hpp` via a small Python script `scripts/gen_config_doc.py` (run by a `docs` target, not by CI).
- A CI lint job runs `markdown-link-check` over all `.md` files; broken cross-links fail CI.
- A `LICENSE` file at the repo root (BSD-3-Clause, matching the prior-art Septentrio driver) is required before sign-off.
- `CONTRIBUTING.md` is **out of scope for v1** (single-user project).

---

### D8. Versioning, ABI, namespace — Q30 = A (default)

**User answer**: **SemVer** with a deliberate 0.x prefix until v1 ships.

- **v0.x.y** — internal API can break between minor versions. Suitable for the user's research workflow (single-user, source-controlled, no external dependents).
- **v1.0.0** — first ABI-stable release. From here, breaking changes bump major; additive changes bump minor; bug fixes bump patch.
- **Post-1.0 ABI stability**: the public `Driver` class uses the **pimpl idiom** (`std::unique_ptr<Impl> impl_;`) so the visible class layout is `sizeof(unique_ptr)` and breaks compile-time only when the public method set itself changes. Other public types (`Config`, `Error`, `Metrics`, record structs) are versioned via header guards `#define ASTERX_RECORD_VERSION_<TYPE> N` and a `static_assert` in the writer; bumping a record's version is a SemVer-major change.
- **Namespace**: everything under `asterx::*`. Sub-namespaces by module: `asterx::transport`, `asterx::sbf`, `asterx::recording`, `asterx::config`, `asterx::time`, `asterx::detail` (private).
- **No `extern "C"` API in v1**. C language interop is deferred to a hypothetical v2; today the only consumer is the user's C++ estimator. Adding a C ABI later is additive (a new `include/asterx/c_api.h` + `libasterx_c.so` shim).

**Rationale**: 0.x lets the architect, the protocol expert, and the reviewer iterate without ABI promises that constrain refactoring. Pimpl is the standard C++ recipe for ABI stability and costs one indirection per call — negligible at the facade level. A namespace-per-module convention keeps the include-graph readable without ALL_CAPS_PREFIXES.

**Architectural implication**:
- `include/asterx/driver.hpp` declares `class Driver { ... private: struct Impl; std::unique_ptr<Impl> impl_; };`. `Driver`'s special member functions are explicitly defaulted in the `.cpp` (`= default;` in `.hpp` would require the Impl to be complete at the include site).
- Record-struct version macros (`ASTERX_RECORD_VERSION_MEASEPOCH = 1`, etc.) are written into the binary file's schema descriptor (per Q9) so a downstream consumer can refuse a file produced by a newer record format.
- A `cmake/AsterXVersion.cmake` (generated via `configure_file`) produces `include/asterx/version.hpp` with `ASTERX_VERSION_MAJOR / MINOR / PATCH / GIT_SHA` constants. The executable's `--version` flag prints these.
- A `tests/abi/` smoke target: a tiny C++ program that includes only public headers, links `asterx_core`, and is compiled separately under each CI cell — guarantees `include/asterx/` is self-contained and doesn't leak private includes.

---

## Approval

By signing below, the user acknowledges that:
- All 30 requirement decisions above accurately reflect their intent for v1 of the AsteRx-i3 D Pro+ Ethernet driver.
- Deviations from architect's default recommendations have been explicitly understood and accepted:
  - **Q3=C** (rates fully config-driven, not hardcoded)
  - **Q4=C** (custom binary format, sacrificing RxTools playback compatibility)
  - **Q17=B** (no console logging sink; only rotating file, with a startup-window stderr exception for fatal errors)
  - **Q20=A** (minimal CLI with no subcommands; `--validate-config` therefore dropped, schema validation runs at startup)
- This document, once signed, **freezes the requirements**. Subsequent architecture design (Phase 2) and implementation (Phase 3) will adhere strictly to these decisions. Changes after sign-off require a new requirements amendment cycle.

- [x] **Approved by user**: lewincentury (per project context email lewincentury@gmail.com)
- **Date signed**: 2026-05-16

Reply with `approve <your-name>` (or `批准 <名字>`) to sign.
