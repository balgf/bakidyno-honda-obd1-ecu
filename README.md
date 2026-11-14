# BakiDyno ECU — `STM32H743VIT6` Standalone Engine Management (Rust)

[![CI](https://github.com/balgf/bakidyno-honda-obd1-ecu/actions/workflows/ci.yml/badge.svg)](https://github.com/balgf/bakidyno-honda-obd1-ecu/actions/workflows/ci.yml)
[![License: MIT OR Apache-2.0](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg)](#license)

Open-source standalone Engine Control Unit written in `no_std` Rust for 1992–1995 Honda Civic OBD1 hardware (`STM32H743VIT6`), featuring:

- **MCU**: `STM32H743VIT6` (`LQFP-100`, `480 MHz` Cortex-M7F, `2 MB` Dual-Bank Flash, `1 MB` RAM)
- **Ignition**: 4x Sequential Smart Coil-on-Plug (`COP1–COP4`, `1-3-4-2`) + `1.2 ms` `TACH_OUT` pulse for the stock cluster tachometer
- **Trigger**: Stock distributor `24/4/1` (`CKP`, `TDC`, `CYP`) + `VSS` decoding (`TIM2` `1 MHz` 32-bit Input Capture)
- **Wideband O2**: Direct onboard **Bosch LSU 4.9** digital controller (`DAC1` + `TIM15` `4 kHz` `Ri` AC + `ADC2` injected sampling + `TIM17` `100 Hz` heater PWM)
- **Vehicle Bus & Aux I/O**: `FDCAN1` (`500 kbps` telemetry `0x640..0x642`), Honda `D7 DLC` `9600 bps` 1-wire serial (`USART1`), `EXTI` Flex Fuel / `ALTF*` capture, and protected aux low-side drivers
- **Tuning Protocols**:
  - `proto-native` (`COBS` + `CRC32` + `postcard` RPC) for `ecu-cli` and the **BakiDyno Tuner** desktop application
  - TunerStudio `msEnvelope_1.0` (`proto-tunerstudio` + [`CivicOBD1-H743-0.1.ini`](./CivicOBD1-H743-0.1.ini)) for universal bench tuning

---

## Safety & Regulatory Disclaimer

> [!WARNING]
> **Experimental Software & Hardware — Use at Your Own Risk**
>
> This project is provided strictly for **bench testing, research, educational, and off-road/motorsport experimentation**. It is **not** certified or homologated for public highway use, emissions compliance, or safety-critical automotive operation.
>
> Improper fuel maps, ignition timing, coil dwell settings, or wiring errors can cause **severe engine damage, fire, toxic exhaust exposure, property damage, serious injury, or death**.
>
> - Always verify hardware safety interlocks (`PB15` `ACTUATOR_OE_N` staying HIGH/disabled through reset until outputs are initialized) on a current-limited bench power supply before connecting live injectors, coils, or fuel pumps.
> - Always confirm distributor base ignition timing with a timing light (via the `D4 SCS` `16.0° BTDC` lock mode) before running an engine under load.
> - This software is provided **"AS IS"**, without warranty of any kind, under the terms of [`LICENSE-MIT`](./LICENSE-MIT) and [`LICENSE-APACHE`](./LICENSE-APACHE).

---

## Bring-Up & Verification Status

| Subsystem / Milestone | Scope & Notes | Current Status |
| :--- | :--- | :--- |
| **Core Engine Math & Host Suite** | Pure `no_std` `24/4/1` trigger decoder, bilinear `16×16` Speed-Density fuel & `1-3-4-2` COP spark, VTEC, LSU 4.9 PI, `FDCAN`, `D7 DLC`, Aux I/O (`45` workspace unit tests) | Verified in host tests & CI |
| **Steps 1–3: `STM32H743` & `RP2040` Trigger Sim** | `25 MHz` HSE → `480 MHz` `SYSCLK`, `PB15 ACTUATOR_OE_N` safe boot, `TIM2` `1 MHz` 32-bit Input Capture (`FullSynced720`), [`rp2040-dizzy-sim`](./tools/rp2040-dizzy-sim/src/main.rs) `24/4/1` + `VSS` generator | Verified on bench hardware |
| **Step 4: `ADC1` DMA Sensors & Switch Inputs** | `ADC1` + `DMA1_CH0` `16-bit` (`8x` oversampling) scanning `MAP`, `TPS`, `ECT`, `IAT`, `VBatt`, `VCC_5V_SNS`, `ELD` via `10K` bench pots + smooth `GP26` RPM sweep + all 7 `SN74HCS14` switch inputs | Verified on bench hardware |
| **Step 5: Real-Time Timer Actuators** | `TIM1` `COP1–4` (`9.5 ms` dwell watchdog), `TIM3` sequential `INJ1–4` (rollover extension `> 50 ms`), `TIM16` `1.2 ms` `TACH_OUT`, `TIM4` `450 Hz` `IACV`/`O2HTC`; `1 MHz` logic-analyzer verified `1-3-4-2` (`17.0° ± 0.1° BTDC`) | Verified on bench hardware |
| **Step 6: USB CDC-ACM, `SPI1` NVM & TunerStudio** | Dual-protocol USB mux (`proto-native` + TunerStudio `msEnvelope_1.0`) + dual-sector ping-pong `SPI1` NOR Flash (`PY25Q64HA` / `W25Q64`) + [`CivicOBD1-H743-0.1.ini`](./CivicOBD1-H743-0.1.ini) Burn persistence | Verified on bench hardware |
| **Bosch LSU 4.9 Wideband Hardware Driver** | `DAC1` (`PA4`/`PA5`), `TIM15` `4 kHz` Nernst `Ri` AC (`PE5`), `ADC2` synchronous injected sampling (`PA6`/`PA7`), `TIM17` `100 Hz` heater PWM (`PB9`) | Closed-loop bench validated |
| **Vehicle Bus, Honda DLC & Aux I/O** | `FDCAN1` `500 kbps` (`0x640..0x642`), `USART1` `9600 bps` Honda `D7 DLC`, `EXTI` (`PB7` `ALTF*`, `PE15` Flex Fuel), aux low-side drivers (`ALTC*`, `IAB*`, `EVAP*`, `PE0`/`PE1`) | Boot & `FDCAN1` loopback verified (`0` faults); vehicle bus harness testing pending |
| **Remaining Roadmap** | `ADC3` `200 kS/s` `BDMA` knock DSP (`PC2_C`/`PC3_C` pad isolation enabled), `SDMMC1` 4-bit MicroSD logger + `LSE` RTC, `SPI1` FRAM (`FM25V02A`) mode, Stage 2 OBD1 PCB & in-car commissioning | Planned |

---

## Workspace & Crate Architecture

| Path | Role |
| :--- | :--- |
| [`crates/ecu-types`](./crates/ecu-types/src/lib.rs) | Canonical `no_std` data types (`CalibrationBlob`, `TelemetrySnapshot`, `FaultFlags`, `SwitchState`) |
| [`crates/ecu-core`](./crates/ecu-core/src/lib.rs) | Pure `no_std` control loops (`trigger`, `fuel`, `ignition`, `vtec`, `sensors`, `wideband`, `can_bus`, `dlc`, `aux_io`, `engine`) |
| [`crates/proto-native`](./crates/proto-native/src/lib.rs) | Framed `COBS` + `CRC32` + `postcard` RPC (`Ping`, `GetTelemetry`, chunked calibration read/write, NVM commit, timing override) |
| [`crates/proto-tunerstudio`](./crates/proto-tunerstudio/src/lib.rs) | TunerStudio `msEnvelope_1.0` Page 1 (`2816 B`) & `OutputChannels` (`24 B`) adapter paired with [`CivicOBD1-H743-0.1.ini`](./CivicOBD1-H743-0.1.ini) |
| [`firmware`](./firmware/src/main.rs) | `STM32H743VIT6` (`embassy-stm32`) bare-metal firmware (`thumbv7em-none-eabihf`) |
| [`tools/ecu-cli`](./tools/ecu-cli/src/main.rs) | Host bench simulation (`sim`, `inspect-cal`, `frame-demo`) and live USB CDC probing (`probe`, `ts-check`) |
| [`tools/rp2040-dizzy-sim`](./tools/rp2040-dizzy-sim/src/main.rs) | `RP2040` bench distributor simulator (`thumbv6m-none-eabi`) generating `24/4/1` `CKP`/`TDC`/`CYP` + `VSS` |

---

## `STM32H743VIT6` Hardware Pin Summary

All pin assignments are defined directly in [`firmware/src/main.rs`](./firmware/src/main.rs) and its driver modules:

| Domain | Peripheral | `STM32H743` Pins | Function |
| :--- | :--- | :--- | :--- |
| **Trigger & Speed** | `TIM2` (`1 MHz` 32-bit IC) | `PA15`, `PA1`, `PB10`, `PB11` | `CKP` (24T), `TDC` (4T), `CYP` (1T), `VSS` |
| **Ignition & Tach** | `TIM1`, `TIM16` | `PE9`, `PE11`, `PE13`, `PE14`, `PB8` | `COP1–4` (`1-3-4-2`), `1.2 ms` `TACH_OUT` |
| **Fuel & PWM** | `TIM3`, `TIM4` | `PC6`, `PC7`, `PB0`, `PB1`, `PD14`, `PD15` | `INJ1–4`, `450 Hz` `IACV` & `O2HTC` |
| **Analog Sensors** | `ADC1` (`DMA1_CH0` 16-bit) | `PC0`, `PC1`, `PA2`, `PA3`, `PC4`, `PA0`, `PC5` | `MAP`, `TPS`, `ECT`, `IAT`, `VBatt`, `VCC_5V_SNS`, `ELD` |
| **Wideband LSU 4.9** | `DAC1`, `ADC2`, `TIM15`, `TIM17` | `PA4`, `PA5`, `PA6`, `PA7`, `PE5`, `PB9` | `Ip` DAC, `1.65 V` `VM`, `UN`, `IP`, `4 kHz` `Ri` AC, heater PWM |
| **Comms & NVM** | `USB_OTG_FS`, `SPI1`, `FDCAN1`, `USART1` | `PA11/PA12`, `PB3/PB4/PD7/PD6`, `PD0/PD1`, `PA9/PA10` | USB CDC-ACM, `SPI1` NOR Flash, `500 kbps` CAN, `9600 bps` `D7 DLC` |
| **Switches & Aux** | `GPIO`, `EXTI7`, `EXTI15` | `PB5`, `PE4`, `PE6–PE8`, `PE10`, `PE12`, `PB14`, `PB7`, `PE15` | `STS`, `VTP*`, `SCS`, `ACS*`, `PSP*`, `BKSW*`, `CLUTCH`, `VTS_FAULT_N`, `ALTF*`, `FLEX` |
| **Relays & Safety** | `GPIO_Out` | `PB15`, `PA8`, `PD3`, `PD5`, `PB12`, `PB13`, `PD8–PD10`, `PE0`, `PE1`, `PE3` | `ACTUATOR_OE_N`, `VTS*`, `FLR`, `MIL`, `FANC*`, `ACC*`, `ALTC*`, `IAB*`, `EVAP*`, spares, heartbeat LED |

---

## Quickstart & Verification Commands

Requires Rust `stable` with `thumbv7em-none-eabihf`, `thumbv6m-none-eabi`, `rustfmt`, and `clippy` (configured in [`rust-toolchain.toml`](./rust-toolchain.toml)).

```bash
# 1. Run host unit tests (45 tests across ecu-core, ecu-types, proto-native, proto-tunerstudio, ecu-cli)
cargo test --workspace

# 2. Check formatting and lints (Google Rust Style Guide, zero warnings)
cargo fmt --all --check
cargo clippy --all-targets -- -D warnings

# 3. Run the closed-loop bench simulation sweep (Crank -> SCS 16° Lock -> Idle -> Cruise -> VTEC WOT)
cargo run -p ecu-cli -- sim
cargo run -p ecu-cli -- inspect-cal
cargo run -p ecu-cli -- frame-demo

# 4. Probe a live ECU over USB CDC-ACM (`proto-native` & TunerStudio `msEnvelope_1.0` + Flash Burn)
cargo run -p ecu-cli -- probe --port /dev/ttyACM0
cargo run -p ecu-cli -- ts-check --port /dev/ttyACM0 --burn-test

# 5. Lint and build STM32H743VIT6 ECU firmware ELF (`thumbv7em-none-eabihf`)
(cd firmware && cargo fmt --check && cargo clippy -- -D warnings && cargo build --release)

# 6. Lint and build RP2040 24/4/1 Distributor Bench Simulator ELF (`thumbv6m-none-eabi`)
(cd tools/rp2040-dizzy-sim && cargo fmt --check && cargo clippy -- -D warnings && cargo build --release)
```

---

## License

Licensed under either of

- Apache License, Version 2.0 ([`LICENSE-APACHE`](./LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT License ([`LICENSE-MIT`](./LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.
