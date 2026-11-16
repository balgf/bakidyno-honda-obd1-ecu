#![no_std]

//! Pure `no_std` deterministic engine control library for the Honda Civic OBD1 `STM32H743VIT6` ECU.
//!
//! Zero hardware HAL dependencies: compiles identically for `x86_64` host tests/simulations
//! and `thumbv7em-none-eabihf` (`Cortex-M7F`) bare-metal firmware.

pub mod aux_io;
pub mod can_bus;
pub mod datalog;
pub mod dlc;
pub mod engine;
pub mod fat32;
pub mod fuel;
pub mod ignition;
pub mod knock;
pub mod math;
pub mod nvm_chip;
pub mod sensors;
pub mod trigger;
pub mod vtec;
pub mod wideband;

pub use engine::{EngineCoordinator, RtControlTargets, RtTriggerEngine, RtTriggerStatus};
