# Dabao Board(Baochip-1x)対応 — スタンドアロン GDB サーバ

[English](./README.md) 日本語

[Dabao Board](https://github.com/betrusted-io/xous-core)(Baochip-1x、RISC-V、Xous OS 上で動作)を
SWD デバッグプローブ化し、USB CDC-ACM ポート上で GDB Remote Serial Protocol を直接話します。
OpenOCD や probe-rs を介さず GDB から `target remote` で接続できます。

他のボードと異なり、ファームウェア本体は**このリポジトリ内の crate ではありません**。xous-core の
フォークにある Xous アプリ `apps-dabao/dabao-gdb` が、本リポジトリの crate(`rust-dap` / `arm-debug` /
`gdb-server-core`)を相対パス依存で取り込む構成です。このディレクトリには解説・ビルドスクリプト・CI 連携を置いています。

| 構成要素 | 場所 |
|---|---|
| プロトコル / ターゲット層(ボード非依存) | [`gdb-server-core`](../../gdb-server-core)、[`arm-debug`](../../arm-debug)、[`rust-dap`](../../rust-dap)(本リポジトリ) |
| Xous アプリ(BIO コアでの SWD、USB セッションループ) | [ciniml/xous-core](https://github.com/ciniml/xous-core) ブランチ `dabao-gdb-server`、`apps-dabao/dabao-gdb` |
| ビルドスクリプト / CI | [`build.sh`](./build.sh)、[`.github/workflows/dabao.yml`](../../.github/workflows/dabao.yml)、[`release.yml`](../../.github/workflows/release.yml) |

## 機能

- USB CDC-ACM(VID:PID `1d50:6197`、product `Dabao`)上の GDB RSP: レジスタ・メモリ・HW ブレーク/ウォッチポイント・
  ステップ・`monitor reset`・マルチコアのスレッド表示・RP2040 フラッシュへの `load`(常駐非同期ローダ)・
  nRF52/nRF54L のフラッシュ書き込みと `monitor approtect` / `erase_all`。
- 対応ターゲット: RP2040、nRF52、nRF54L(DPIDR で自動判別)。
- SWD は Baochip の BIO コア 1 個にオフロードし SWCLK 8 MHz(実測: RAM 読み ~225 KiB/s、RAM 書き ~207 KiB/s、
  RP2040 `load` ~58 KB/s)。
- RTT: `monitor rtt scan | attach <addr> | dump`。ターゲット走行中は RTT up チャネルのテキストを GDB コンソール出力
  (`O` パケット)として転送し、GDB の画面にリアルタイム表示(~80 KiB/s)。このボードには RTT 用の独立 CDC ポートは
  ありません(Baochip の USB コアは IN 4 本 + OUT 4 本しかなく HID + RSP ポートで使い切るため)。down 方向も非対応。
- `monitor diag` でリンク統計(SWD ACK ヒストグラム、RSP キューの高水位)を表示。

## 配線

| Dabao ピン | SWD 信号 |
|---|---|
| PB1 | SWCLK |
| PB2 | SWDIO |
| PB3 | nRESET(ターゲット RUN / リセット) |
| GND | GND |

PB1〜PB5 は Dabao のヘッダに出ており、ブートローダの予約(PB13/PB14 = コンソール UART、PC13 = USB SE0 / `PROG`)と
干渉しません。ピン定義は `apps-dabao/dabao-gdb/src/main.rs` 冒頭の定数です。

RSP がボード唯一の USB シリアルを専有するため、Xous のログコンソールは物理 UART です:
**PB14 = TX、PB13 = RX、1 Mbaud**(例: FT232H 経由で `screen /dev/ttyUSB0 1000000`)。

## ビルド

必要なもの: Xous toolkit のリリースが存在する stable Rust(ビルドスクリプトが `rustc --version` を確認。1.97.x / 1.98.0 は動作確認済み)、
`git`、初回のみネットワーク(Xous の `xtask` が `riscv32imac-unknown-xous-elf` の標準ライブラリをダウンロード)。

```shell
# rust-dap のチェックアウトから
./boards/dabao/build.sh            # ../xous-core(ciniml、dabao-gdb-server)が無ければ clone
./boards/dabao/build.sh --flash    # …続けて apps-dabao/dabao-gdb/flash.py で書き込み(FT232H 必要)
```

スクリプトの動作:

1. 本リポジトリの**隣**に xous-core のチェックアウトがあることを確認(`../xous-core`。`XOUS_CORE=/path` または
   `--xous-core PATH` で変更、ブランチは `XOUS_BRANCH`)。`apps-dabao/dabao-gdb/Cargo.toml` の相対パス依存は
   `../../../rust-dap/...` を指すため、2 つのチェックアウトは兄弟ディレクトリである必要があります(`--rust-dap PATH` を
   渡した場合はシンボリックリンクを作成)。
2. sysroot にターゲットが無ければ Xous toolkit をインストール(`cargo xtask install-toolkit --force --no-verify`)。
3. xous-core で `cargo xtask dabao dabao-gdb` を実行。
4. `loader.uf2` / `xous.uf2` / `apps.uf2` と `dabao-gdb` ELF を `boards/dabao/dist/` にコピー(`--out DIR` で変更)。

手動で行う場合:

```shell
git clone -b dabao-gdb-server https://github.com/ciniml/xous-core ../xous-core
cd ../xous-core
cargo xtask install-toolkit --force --no-verify     # rustc のバージョンごとに一度
cargo xtask dabao dabao-gdb
ls target/riscv32imac-unknown-xous-elf/release/{loader,xous,apps}.uf2
```

## 書き込み

3 つの UF2 は組で使います(フォークは `xous.uf2` 側の USB サービスにも修正を含む)。手動の場合は
`loader.uf2` → `xous.uf2` → `apps.uf2` の順に書き込んでください。

- **手動**: `PROG` を押しながら USB 接続 → `BAOCHIP` ボリュームが見える → UF2 をコピー → `PROG` 再押下(またはブートローダの
  `boot` コマンド)で起動。
- **スクリプト**(`build.sh --flash`、または xous-core の `apps-dabao/dabao-gdb/flash.py`): FT232H(RTS# / ADBUS2 をリセット
  ボタンに配線、GND 共通)でリセットをパルス → boot1 ブートローダの USB コンソール(`Baochip-1x`、`1d50:6196`)を待つ →
  `uf2` コマンドで UF2 を転送 → `boot`。boot1 で一度 `bootwait` を有効化しておく必要があります(`PROG` で boot1 に入り
  `bootwait` と打つ)。Python 3 + pyserial のみで動作。

## 使い方

```shell
gdb-multiarch -ex 'set remotetimeout 10' \
              -ex 'target remote /dev/serial/by-id/usb-Baochip_Dabao_*-if02' \
              -ex 'set architecture armv4t' -ex 'info registers'
```

- プローブは最初のセッション前にターゲットへ接続して halt するので、GDB は停止中のコアに attach します。
  `monitor help` でモニタコマンド一覧。
- ModemManager は停止するか、`1d50:6197` に `ID_MM_DEVICE_IGNORE=1` の udev ルールを追加してください(列挙直後に CDC ポートを
  掴まれます)。
- `monitor reset` 後は GDB のレジスタキャッシュが古いままなので `maintenance flush register-cache`(または `stepi`)を。
- RTT: `monitor rtt scan` → `monitor rtt attach <addr>` → `continue` で走行中の出力が GDB コンソールに表示。halt 中に溜まった
  分は `monitor rtt dump`。

## 既知の制約

- USB CDC が 1 本のみ: RTT 端末ポートや UART ブリッジはありません(エンドポイント不足、「機能」参照)。RTT down 方向も非対応。
- RP2350、SWJ/JTAG、CMSIS-DAP プロトコルは未実装(GDB サーバ専用)。
- Xous 側コードは上流 betrusted-io ではなく `ciniml/xous-core` フォークで保守(RSP に必要な USB スタック修正を含むため)。
