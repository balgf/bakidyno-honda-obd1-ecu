# Rust Coding Standards — Google Rust Style Guide

All Rust code in this workspace (`ecu-types`, `ecu-core`, `proto-native`, `proto-tunerstudio`, `firmware`, and `tools`) MUST strictly adhere to the [Google Rust Style Guide](https://google.github.io/styleguide/rustguide.html):

1. **Imports**: Never use wildcard glob imports (`use foo::*;`). Explicitly import items or reference the module path.
2. **Formatting**: Hard column width limit of 100 (`max_width = 100`). Use lowercase hex literals (`0x1a2b`).
3. **Panic Hygiene**: No bare `.unwrap()` in production or firmware code. Always return a typed `Result` or use descriptive `.expect("explanation of invariant")`.
4. **Ownership**: Favor unique ownership and borrowing (`&T`, `&mut T`). Avoid `Rc`/`Arc` unless strictly necessary and justified.
5. **Traits**: Define traits strictly for generics (`T: Trait`), abstract returns (`impl Trait`, `dyn Trait`), or extension traits (`~Ext`). Do not use traits for grouping arbitrary methods.
6. **Unsafe**: Minimize unsafe blocks. Every `unsafe` block MUST be preceded by a `// SAFETY: <justification>` comment. Every `unsafe fn` must document `# Safety`.
7. **Verification**: All code must compile cleanly with `cargo test` and `cargo clippy --all-targets -- -D warnings` with zero warnings.
