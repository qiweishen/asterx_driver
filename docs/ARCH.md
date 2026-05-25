# asterx_driver — slim architecture

A single-binary recorder for the **Septentrio AsteRx-i3 D Pro+**. Goal:
record raw IMU + dual-antenna GNSS observations + receiver health to disk
over Ethernet, with a small set of receiver-side configuration commands.

For full requirements rationale see `docs/architecture/00-requirements.md`
(signed, kept for archival; v1 ships a slim subset of those).

## What we record

| Block             | Block ID | Rate (default) | What it is                                  |
|-------------------|----------|----------------|---------------------------------------------|
| `ExtSensorMeas`   | 4050     | 200 Hz         | IMU accel + gyro                            |
| `MeasEpoch`       | 4027     | 10 Hz          | Raw pseudorange / carrier / Doppler / CN0 (per-antenna via `AntennaID`) |
| `MeasExtra`       | 4000     | 10 Hz          | Per-signal extras (lock time, σ, etc.)      |
| `AuxAntPositions` | 5942     | 10 Hz          | Main↔Aux1 baseline                          |
| `AttEuler`        | 5938     | 10 Hz          | GNSS attitude (yaw/pitch/roll)              |
| `AttCovEuler`     | 5939     | 10 Hz          | Covariance of `AttEuler`                    |
| `ReceiverStatus`  | 4014     | 1 Hz           | CPU/RAM/temperature                         |
| `QualityInd`      | 4082     | 1 Hz           | Per-signal-source quality indicators        |
| `ReceiverTime`    | 5914     | 1 Hz           | Receiver vs system clock                    |

All blocks are written **verbatim** as raw SBF bytes to a rotating `.sbf`
file. We do NOT decode the bodies. RxTools, `pysbf`, or any SBF-aware
post-processor can read the output.

## Data flow

```
                    main(): single thread, blocking I/O
   ┌─────────────────────────────────────────────────────────────────┐
   │                                                                 │
   │   1. TcpClient ctrl ── connect ─→ AsteRx host:28784             │
   │                       login,user,pass                           │
   │                       setIPServerSettings IPS1 28785            │
   │                       setMeasMeasurements AllAntennas           │
   │                       setImuOrientation ...                     │
   │                       setSBFOutput Stream1..4 IPS1 ...          │
   │                                                                 │
   │   2. TcpClient data ── connect ─→ AsteRx host:28785             │
   │                                                                 │
   │   3. loop:                                                      │
   │        data.read_some(buf, 10s)                                 │
   │        recorder.feed(buf, n):                                   │
   │            SbfFrameSync::feed → vector<Frame>                   │
   │            for each Frame: reconstruct 8B header + body         │
   │                            → fwrite to current .sbf             │
   │                            → rotate if >1 GiB or >1 h           │
   │                                                                 │
   │   4. SIGINT: stop loop, stop_streams, close file, exit          │
   └─────────────────────────────────────────────────────────────────┘
```

## Module layout

```
src/
├── main.cpp                       # CLI, signal handling, top-level loop
├── tcp_client.{hpp,cpp}           # blocking BSD socket (no Asio, no TLS)
├── receiver_config.{hpp,cpp}      # login + setSBFOutput + setImuOrientation
├── sbf_recorder.{hpp,cpp}         # frame_sync + rotating file writer
└── sbf/
    ├── crc16_xmodem.{hpp,cpp}     # constexpr CRC-CCITT-XMODEM table
    └── frame_sync.{hpp,cpp}       # 4-state SBF byte machine
```

## What's explicitly out of scope

- TLS (port 28783) — plaintext only.
- mDNS / `.local` discovery — config-driven host/IP only.
- Asynchronous I/O — single blocking thread.
- Multi-threaded pipelines, lockfree queues, subscriber API.
- Custom on-disk format (`.astrx`, schema descriptors) — raw `.sbf` only.
- Reconnect FSM, credential-precedence layering, pimpl ABI, installable library.

If any of these become necessary later, they are additive — not a v1 concern.

## Dependencies

System packages (Ubuntu 20.04):
- `libyaml-cpp-dev` (0.6.2)
- `libspdlog-dev` (1.5.0)
- A C++17 toolchain (GCC 9.4+, Clang 10+)

Fetched at build time (test-only):
- GoogleTest v1.14.0 (FetchContent)

## Build (in your Docker container)

```bash
apt-get install -y libyaml-cpp-dev libspdlog-dev cmake ninja-build g++
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure   # runs CRC + frame_sync unit tests
```

## Run

```bash
cp config.example.yaml config.yaml
# edit config.yaml: at minimum set connection.host and connection.password
./build/bin/asterx_driver --config config.yaml
```

The driver creates `./recordings/asterx-YYYYMMDDTHHMMSSZ-N.sbf` on startup
and rotates when the current file passes 1 GiB or has been open 1 hour.
SIGINT stops the loop cleanly, tears down receiver streams, closes the file.

## Post-processing the output

Any SBF-aware tool reads `.sbf` files. Two common paths:
- **RxTools** (Septentrio's free GUI on Windows): `bin2asc` to dump human-readable text.
- **Python**: `pip install pysbf` or use the existing CRC/parser code in
  Septentrio's open-source ROS driver
  (<https://github.com/septentrio-gnss/septentrio_gnss_driver>, BSD-3-Clause).

## Receiver reference

`docs/AsteRx-i3_D_Pro__Firmware_v1_5_2_Reference_Guide.pdf` — authoritative
SBF block definitions and command syntax.

`docs/architecture/03-sbf-parser-design.md` — explanation of the SBF wire
format and our frame synchronizer, retained because it documents
non-obvious parts of the protocol.
