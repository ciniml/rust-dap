#!/usr/bin/env bash
# Build the Dabao Board GDB-server firmware (Xous app `dabao-gdb`).
#
# The firmware lives in a fork of xous-core and pulls rust-dap's crates in
# through relative path dependencies (../../../rust-dap/...), so xous-core must
# be checked out next to this repository. This script sets that up, installs
# the Xous toolkit if needed, runs the xtask build, and collects the images.
#
#   ./boards/dabao/build.sh [--xous-core PATH] [--rust-dap PATH] [--out DIR]
#                           [--no-clone] [--flash [FLASH ARGS...]]
#
# Environment: XOUS_CORE (default ../xous-core next to this repo),
#              XOUS_REPO (default https://github.com/ciniml/xous-core),
#              XOUS_BRANCH (default dabao-gdb-server).
set -euo pipefail

here=$(cd "$(dirname "$0")" && pwd)
rust_dap=$(cd "$here/../.." && pwd)
xous_core=${XOUS_CORE:-$(dirname "$rust_dap")/xous-core}
xous_repo=${XOUS_REPO:-https://github.com/ciniml/xous-core}
xous_branch=${XOUS_BRANCH:-dabao-gdb-server}
out=$here/dist
clone=1
flash=0
flash_args=()

while [ $# -gt 0 ]; do
  case "$1" in
    --xous-core) xous_core=$2; shift 2 ;;
    --rust-dap)  rust_dap=$(cd "$2" && pwd); shift 2 ;;
    --out)       out=$2; shift 2 ;;
    --no-clone)  clone=0; shift ;;
    --flash)     flash=1; shift; flash_args=("$@"); break ;;
    -h|--help)   sed -n '2,16p' "$0"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

target_triple=riscv32imac-unknown-xous-elf
release=target/$target_triple/release

# 1. xous-core checkout
if [ ! -d "$xous_core/.git" ]; then
  if [ "$clone" = 1 ]; then
    echo ">> cloning $xous_repo ($xous_branch) into $xous_core"
    git clone --branch "$xous_branch" "$xous_repo" "$xous_core"
  else
    echo "xous-core checkout not found at $xous_core (pass --xous-core or unset --no-clone)" >&2
    exit 1
  fi
fi
xous_core=$(cd "$xous_core" && pwd)

# The app's Cargo.toml expects rust-dap at <xous-core>/../rust-dap.
expected_rust_dap=$(dirname "$xous_core")/rust-dap
if [ ! -e "$expected_rust_dap" ]; then
  echo ">> linking $expected_rust_dap -> $rust_dap"
  ln -s "$rust_dap" "$expected_rust_dap"
elif [ "$(cd "$expected_rust_dap" && pwd -P)" != "$(cd "$rust_dap" && pwd -P)" ]; then
  echo "warning: $expected_rust_dap is not this checkout ($rust_dap); the build will use the former" >&2
fi

# 2. Xous toolkit (std for the custom target) — downloaded per rustc version.
sysroot=$(rustc --print sysroot)
if [ ! -d "$sysroot/lib/rustlib/$target_triple" ]; then
  echo ">> installing Xous toolkit for $(rustc --version) into $sysroot"
  (cd "$xous_core" && cargo xtask install-toolkit --force --no-verify)
fi

# 3. Build
echo ">> cargo xtask dabao dabao-gdb (in $xous_core)"
(cd "$xous_core" && cargo xtask dabao dabao-gdb --no-verify)

# 4. Collect
mkdir -p "$out"
for f in loader.uf2 xous.uf2 apps.uf2 dabao-gdb; do
  cp "$xous_core/$release/$f" "$out/"
done
echo ">> images in $out:"
ls -la "$out"

# 5. Optional flash (FT232H reset + boot1 USB console; see flash.py --help)
if [ "$flash" = 1 ]; then
  echo ">> flashing"
  exec "$xous_core/apps-dabao/dabao-gdb/flash.py" "${flash_args[@]}" \
       "$out/loader.uf2" "$out/xous.uf2" "$out/apps.uf2"
fi
