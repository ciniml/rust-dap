use std::process::Command;

fn main() {
    let stdout = Command::new("git")
        .args(["describe", "--always", "--abbrev=40", "--dirty=+"])
        .output()
        .map(|output| output.stdout);

    if let Ok(stdout) = stdout {
        let git_hash = String::from_utf8(stdout).expect("git output not utf8");
        println!("cargo:rustc-env=GIT_FW_VER=git:{}", git_hash.trim());
    }
}
