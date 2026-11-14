fn main() {
    if let Some(dir) = std::env::var_os("CARGO_MANIFEST_DIR") {
        println!("cargo:rustc-link-search={}", dir.to_string_lossy());
    }
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
