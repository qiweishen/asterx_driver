# asterx_driver

A small C++ driver that records raw IMU + dual-antenna GNSS observations from
a **Septentrio AsteRx-i3 D Pro+** receiver over Ethernet (plaintext TCP) and
writes raw SBF bytes to a rotating `.sbf` file. Plus the receiver-side
configuration commands needed to bring the streams up.

Target platform: Ubuntu 20.04.6 LTS, C++17, pure CMake. **No ROS.**

## Build

System packages (one-time):

```bash
sudo apt install -y \
    build-essential cmake ninja-build pkg-config \
    libyaml-cpp-dev libspdlog-dev
```

Configure + build:

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

Unit tests (CRC + SBF frame synchronizer):

```bash
ctest --test-dir build --output-on-failure
```

## Run

```bash
cp config.example.yaml config.yaml
# edit config.yaml — at minimum set connection.host and connection.password
./build/bin/asterx_driver --config config.yaml
```

Output goes to `./recordings/asterx-YYYYMMDDTHHMMSSZ-N.sbf`. Files rotate
when they pass 1 GiB or have been open for 1 hour, whichever comes first.
SIGINT (Ctrl-C) stops the loop cleanly: stops the receiver streams, closes
the file, exits.

## What's recorded

Raw SBF bytes — exactly what the receiver emits, after CRC validation. No
custom format, no transcoding. Decode later with RxTools, `pysbf`, or by
linking the SBF parsers out of Septentrio's open-source ROS driver
(<https://github.com/septentrio-gnss/septentrio_gnss_driver>, BSD-3-Clause).

Default SBF streams (configurable in `config.yaml`):

| Block             | Default rate | What it is                        |
|-------------------|--------------|-----------------------------------|
| `ExtSensorMeas`   | 200 Hz       | IMU accel + gyro                  |
| `MeasEpoch`       | 10 Hz        | Raw pseudorange / carrier / Doppler / CN0, both antennas |
| `MeasExtra`       | 10 Hz        | Per-signal lock time, σ, etc.     |
| `AuxAntPositions` | 10 Hz        | Main↔Aux1 baseline                |
| `AttEuler`        | 10 Hz        | GNSS attitude (yaw/pitch/roll)    |
| `AttCovEuler`     | 10 Hz        | Covariance of `AttEuler`          |
| `ReceiverStatus`  | 1 Hz         | CPU / RAM / temperature           |
| `QualityInd`      | 1 Hz         | Per-signal-source quality         |
| `ReceiverTime`    | 1 Hz         | Receiver vs system clock          |

See `ARCH.md` for the design summary and `docs/architecture/00-requirements.md`
for the (signed) requirements that motivated the choices.

## CLI

```
asterx_driver --config <path.yaml> [--log-level <level>]
```

- `--config <path>` — required, YAML config file.
- `--log-level <level>` — `trace|debug|info|warn|err|critical` (overrides config).

Exit codes: `0` clean shutdown, `1` runtime error after startup, `2` config
error, `3` CLI usage error.

## Layout

See `ARCH.md` for the module breakdown. Briefly:

```
src/
├── main.cpp                  — CLI + signal handling + recorder loop
├── tcp_client.{hpp,cpp}      — blocking BSD socket
├── receiver_config.{hpp,cpp} — Septentrio ASCII command sequence
├── sbf_recorder.{hpp,cpp}    — frame_sync + rotating .sbf writer
└── sbf/                      — CRC-CCITT-XMODEM + frame synchronizer
```

## License

BSD-3-Clause (see `LICENSE`).
