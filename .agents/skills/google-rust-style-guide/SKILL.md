---
name: google-rust-style-guide
description: >-
  Enforces the official Google Rust Style Guide (https://google.github.io/styleguide/rustguide.html)
  across Rust codebases. Use when authoring, reviewing, refactoring, or formatting Rust code,
  structuring crates and modules, designing errors and traits, handling unsafe code, or configuring
  Clippy and rustfmt.
---

# Google Rust Style Guide

This skill enforces the standards and conventions defined in the official
[Google Rust Style Guide](https://google.github.io/styleguide/rustguide.html).
Google treats code style as a software engineering tool: code should be optimized for
readability, long-term maintainability, and large-scale automated refactoring (LSCs).

---

## 1. Core Philosophy: Code is for the Reader

* **Clarity over Cleverness**: Write code that is easy for future engineers to read and review.
* **Explicit over Implicit**: Avoid magic macros or obscure abstractions where simple, explicit
  code suffices.
* **Zero Warnings**: Production code, tests, and tools must compile cleanly under
  `cargo clippy --all-targets -- -D warnings` with zero warnings.

---

## 2. Formatting & Visual Layout (`rustfmt`)

Google standardizes formatting using `rustfmt` with specific configuration flags:

* **Line Length**: Hard limit of **100 columns** (`max_width = 100`).
* **Hexadecimal Literals**: Use **lowercase** letters for hexadecimal values (`hex_literal_case = Lower`):
  * Good: `0xdead_beef`, `0x1a2b`
  * Bad: `0xDEAD_BEEF`, `0x1A2B`
* **Imports (`use` Statements)**:
  * **No Wildcard Glob Imports**: Prohibit `use foo::*;` (exceptions: test modules `#[cfg(test)]`
    and preludes specifically designed for it).
  * **One Module per `use`**: Group items from the same module cleanly, but do not nest `::` inside braces:
    * Good: `use embassy_time::{Duration, Instant, Timer};`
    * Bad: `use embassy_time::{Duration, timer::{Instant, Timer}};`
  * **Locally-Scoped Trait Imports**: When a trait is only needed for its methods inside a single
    function, import it *inside* the function body rather than polluting module scope:
    ```rust
    fn process_payload(buf: &mut [u8]) {
        use byteorder::{ByteOrder, LittleEndian};
        LittleEndian::write_u16(&mut buf[0..2], 42);
    }
    ```

---

## 3. Crate Architecture & Module Layout

* **Small, Single-Purpose Crates**: Prefer fine-grained crates with narrow, focused responsibilities
  (e.g., our workspace split: `ecu-types`, `ecu-core`, `proto-native`, `firmware`).
* **Flat File Structure**:
  * Within a single crate, keep files in a flat directory layout.
  * If a crate feels large enough to require deep subdirectories, treat it as a sign that the
    crate should be split into smaller, independent crates.
* **Submodules for Visibility, Not Organization**:
  * Do not create submodules merely as folders to file away code.
  * Use submodules strictly for encapsulation and controlling item visibility (`pub(crate)`, `pub(super)`).

---

## 4. Naming Conventions

Follow standard Rust naming conventions strictly:

| Element | Convention | Example |
| :--- | :--- | :--- |
| **Types, Structs, Enums** | `PascalCase` | `CalibrationBlob`, `EngineCoordinator` |
| **Traits** | `PascalCase` | `SensorDecoder`, `IgnitionScheduler` |
| **Extension Traits** | `~Ext` suffix | `TimerExt`, `AdcChannelExt` |
| **Functions, Methods** | `snake_case` | `calculate_dwell()`, `sample_ntc()` |
| **Variables, Fields** | `snake_case` | `manifold_pressure_kpa`, `spark_advance_deg` |
| **Constants, Statics** | `SCREAMING_SNAKE_CASE` | `HONDA_FIRING_ORDER`, `MAX_RPM_LIMIT` |
| **Modules, Crates** | `snake_case` | `sensors`, `ecu_core` |
| **Lifetimes** | Descriptive names | `'cal`, `'buf`, `'ctx` (avoid `'a` unless trivial 1:1) |

### Method Naming by Ownership
* `as_*`: Inexpensive, borrowed conversion (`as_slice(&self) -> &[u8]`).
* `to_*`: Potentially expensive or cloning conversion (`to_bytes(&self) -> [u8; 32]`).
* `into_*`: Consuming conversion (`into_inner(self) -> T`).

---

## 5. Ownership, Borrowing & Allocation

* **Default to Unique Ownership**: Prefer standard ownership (`T`) and borrowing (`&T`, `&mut T`).
* **Avoid `Rc` and `Arc` by Default**: Do not use reference-counting smart pointers unless
  unique ownership or borrowing would introduce unreasonable architectural complexity. Every
  use of `Rc`/`Arc` must be justifiable in code review.
* **Accept Flexible Borrowed Slices**: Functions accepting immutable data should accept borrowed
  views rather than owned or smart-pointer types:
  * Pass `&str` instead of `&String`.
  * Pass `&[T]` instead of `&Vec<T>`.
  * Pass `&T` instead of `&Box<T>`.
* **Zero-Allocation in Embedded (`no_std`)**:
  * In embedded and hard real-time code, never allocate on the heap during cyclic execution.
  * Use fixed-size buffers, arrays, or bounded queues (`heapless`).

---

## 6. Error Handling & Panic Hygiene

Google maintains a strict distinction between **recoverable failures** and **program bugs**:

* **Recoverable Errors**: Return `Result<T, E>` with a strongly-typed, descriptive error enum.
* **No Bare `.unwrap()` in Production**:
  * Never use bare `.unwrap()` in production library or firmware code.
  * Use **`.expect("detailed description of the invariant that guarantees success")`** when an
    error is mathematically or structurally impossible:
    ```rust
    // Good: documents the exact invariant
    let val = table.get(idx).expect("idx clamped to table.len() - 1 above");
    ```
* **No `panic::catch_unwind`**: Do not rely on catching unwinding panics for control flow.
* **Validate at Boundaries**: Validate untrusted input (e.g., UART packets, ADC sensor spikes)
  at the ingestion boundary; internal domain logic can then safely assume validated invariants.

---

## 7. Traits & Interface Design

* **When to Define a Trait**:
  Only define a new trait for:
  1. **Generic programming** (`fn run<T: EngineDriver>(d: &mut T)`).
  2. **Abstract trait objects** (`&mut dyn Trait` or `impl Trait`).
  3. **Extension traits** (`pub trait SliceExt { ... }`).
  * **Never** define a trait merely to group methods or mimic an OOP class hierarchy.
* **Implement Standard Library Traits**:
  Prefer implementing standard traits rather than inventing custom method names:
  * Implement `Default` instead of an ad-hoc `empty()` method.
  * Implement `core::fmt::Display` instead of `to_string()`.
  * Implement `core::convert::From` / `TryFrom` instead of custom parse functions.
  * Derive `Debug`, `Clone`, `Copy`, `PartialEq`, `Eq` wherever semantically valid.

---

## 8. Unsafe Code Hygiene

* **Minimize and Encapsulate**: Confine `unsafe` to the smallest possible boundary and expose
  a completely safe public API.
* **Mandatory `// SAFETY:` Comments**: Every single `unsafe` block must be immediately preceded
  by a comment explaining why the operation is guaranteed safe:
  ```rust
  // SAFETY: `SYSCFG.pmcr` bits PC2SO/PC3SO only disconnect PC2/PC3 from ADC3_INP0/1
  // to connect the analog switches to the outer LQFP pins. No concurrent access.
  unsafe {
      let pmcr = stm32_metapac::SYSCFG.pmcr();
      pmcr.modify(|w| {
          w.set_pc2so(true);
          w.set_pc3so(true);
      });
  }
  ```
* **Document Invariants with `# Safety`**: Every `unsafe fn` must have a `# Safety` section in
  its documentation specifying the caller's obligations.

---

## 9. Comments & Inline Argument Labeling

* **Documentation Comments**: Use `///` for public items and `//!` for module/crate headers.
* **Inline Argument Comments**: When passing raw boolean or numeric literals to functions, use
  inline C-style comments to clarify the parameter name:
  ```rust
  enable_channel(/* channel= */ 1, /* invert= */ false);
  ```

---

## 10. Automated Style Verification Runbook

Before submitting or committing any Rust code, run the following verification chain:

```bash
# 1. Check code formatting against 100-col and style rules
cargo fmt --check

# 2. Strict Clippy check with zero warnings allowed
cargo clippy --all-targets -- -D warnings

# 3. Run all unit and integration tests
cargo test
```
