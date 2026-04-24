# extra-examples

[![CI](https://github.com/copper-project/extra-examples/actions/workflows/ci.yml/badge.svg)](https://github.com/copper-project/extra-examples/actions/workflows/ci.yml)
![GitHub last commit](https://img.shields.io/github/last-commit/copper-project/extra-examples)
![](https://img.shields.io/badge/Rust-1.95+-orange.svg)
[![dependency status](https://deps.rs/repo/github/copper-project/extra-examples/status.svg)](https://deps.rs/repo/github/copper-project/extra-examples)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)
[![Copper](https://img.shields.io/badge/Copper-master-b87333)](https://github.com/copper-project/copper-rs)

External Copper robot and application examples.

This repository is an application-level satellite of
[`copper-project/copper-rs`](https://github.com/copper-project/copper-rs). It
keeps heavier end-to-end demos out of the main Copper repository so the core
runtime workspace can stay focused on framework code, canonical examples, and
CI surfaces that matter to Copper itself.

This repository currently contains:

- `examples/cu_flight_controller`: quadcopter flight controller demo
- `examples/cu_rp_balancebot`: Raspberry Pi balance bot demo and sim
- `examples/cu_human_pose`: pose estimation demo with GStreamer camera input
- `examples/cu_elrs_bdshot_demo`: RP2350 ELRS + BDShot embedded demo
- `examples/cu_feetech_demo`: Feetech servo arm demo
- `examples/cu_gnss_ublox_demo`: GNSS receiver demo

The Copper dependencies intentionally track the `master` branch of
[`copper-project/copper-rs`](https://github.com/copper-project/copper-rs). CI
can be triggered from the main Copper repository with the `copper-rs-master`
repository dispatch event.

## Links

- Main Copper runtime and SDK: [`copper-project/copper-rs`](https://github.com/copper-project/copper-rs)
- Copper documentation: <https://copper-project.github.io/copper-rs>
- Copper book: <https://copper-project.github.io/copper-rs-book/>
- Copper component catalog: <https://cdn.copper-robotics.com/catalog/index.html>

## Checks

```bash
just pr-check
```

`just pr-check` runs formatting verification, the existing host/embedded compile
smoke checks, and the host-side unit tests that are stable in CI.

`just check` remains available when you only want the compile-smoke pass. The
host checks cover the Linux-targeted apps. `cu-human-pose` needs GStreamer
development packages installed locally. The embedded smoke checks use cross-target
`cargo check` against the same RP2350 and STM32H7 targets exercised by these
demos.

## License

This repository is licensed under the Apache License, Version 2.0.
