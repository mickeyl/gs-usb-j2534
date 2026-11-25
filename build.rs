//! 32-bit Windows: `extern "system"` functions are stdcall and the linker decorates them
//! (`_PassThruOpen@8`). J2534 applications call `GetProcAddress("PassThruOpen")`, so export the
//! plain names as aliases. On x64 there is a single calling convention and nothing to do.

fn main() {
    let target = std::env::var("TARGET").unwrap_or_default();
    if !target.starts_with("i686-") || !target.contains("windows-msvc") {
        return;
    }
    // (name, argument bytes): every argument of the 04.04 API is 4 bytes wide on x86.
    const EXPORTS: &[(&str, u32)] = &[
        ("PassThruOpen", 8),
        ("PassThruClose", 4),
        ("PassThruConnect", 20),
        ("PassThruDisconnect", 4),
        ("PassThruReadMsgs", 16),
        ("PassThruWriteMsgs", 16),
        ("PassThruStartPeriodicMsg", 16),
        ("PassThruStopPeriodicMsg", 8),
        ("PassThruStartMsgFilter", 24),
        ("PassThruStopMsgFilter", 8),
        ("PassThruSetProgrammingVoltage", 12),
        ("PassThruReadVersion", 16),
        ("PassThruGetLastError", 4),
        ("PassThruIoctl", 16),
    ];
    for (name, bytes) in EXPORTS {
        println!("cargo:rustc-cdylib-link-arg=/EXPORT:{name}=_{name}@{bytes}");
    }
    println!("cargo:rerun-if-changed=build.rs");
}
