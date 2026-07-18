use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;
use std::process::Command;

/// Embed the git revision (short SHA, `-dirty` if the tree has uncommitted
/// changes) as `GIT_REV`, so the firmware can report it via DAP_Info(0x09)
/// Product Firmware Version. Falls back to "unknown" outside a git checkout.
fn emit_git_rev() {
    let rev = Command::new("git")
        .args(["rev-parse", "--short=7", "HEAD"])
        .output()
        .ok()
        .filter(|o| o.status.success())
        .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
        .filter(|s| !s.is_empty());
    let git_rev = match rev {
        Some(rev) => {
            let dirty = Command::new("git")
                .args(["status", "--porcelain"])
                .output()
                .ok()
                .filter(|o| o.status.success())
                .map(|o| !o.stdout.is_empty())
                .unwrap_or(false);
            if dirty {
                format!("{rev}-dirty")
            } else {
                rev
            }
        }
        None => "unknown".to_string(),
    };
    println!("cargo:rustc-env=GIT_REV={git_rev}");
    // Re-run when the checked-out commit moves. `.git/HEAD` only changes on a
    // branch switch, so also watch `.git/logs/HEAD`, which the reflog updates
    // on every commit/reset — otherwise GIT_REV stays cached across commits.
    for path in ["HEAD", "logs/HEAD"] {
        if let Some(p) = Command::new("git")
            .args(["rev-parse", "--git-path", path])
            .output()
            .ok()
            .filter(|o| o.status.success())
            .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
            .filter(|s| !s.is_empty())
        {
            println!("cargo:rerun-if-changed={p}");
        }
    }
}

fn main() {
    emit_git_rev();

    // Put `memory.x` in the linker search path. The linker used to find it in
    // the invocation directory, but in a workspace rustc runs from the
    // workspace root, so it must be provided explicitly.
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
}
