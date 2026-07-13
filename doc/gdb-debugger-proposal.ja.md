# スタンドアロン GDB デバッガファームウェア 設計方針

対象: rust-dap を土台に、MCU 単体で USB-UART(CDC)経由の GDB プロトコルを受け、
SWD で接続した **RP2040 ターゲット**をデバッグできるファームウェア。
プローブ側 MCU も当面 RP2040 とする。

ステータス: 方針検討(未実装)。実装は rust-dap 0.3 の `DapTransport` を前提とする。

---

## 0. 用語の整理(重要): GDB MI ではなく RSP

「GDB MI コマンドを受け取る」という要件について、まず区別が必要:

- **GDB/MI (Machine Interface)**: GDB *本体*と IDE 等のフロントエンドの間の
  プロトコル(`-exec-continue` 等)。GDB プロセスの上位に位置する。
- **GDB Remote Serial Protocol (RSP)**: GDB *本体*とリモートの実行環境
  (gdbserver / デバッグプローブ)の間のプロトコル(`$...#xx` パケット、
  `g`/`G`/`m`/`M`/`c`/`s`/`Z`/`z` 等)。

**デバッグプローブのファームウェアが実装すべきは RSP** である。ユーザーは
`gdb-multiarch` で `target extended-remote /dev/ttyACMx` としてプローブに接続し、
GDB が RSP を話す。MI はさらにその上(GDB↔IDE)なので、IDE 連携が必要なら
ホスト側の GDB がそのまま MI を提供する。したがってファーム側は RSP に集中する。

これは **Black Magic Probe (BMP)** と同じアーキテクチャ:プローブ自身が
gdbserver になり、OpenOCD/pyOCD/probe-rs のような別デーモンを不要にする。
本提案は「Rust による no_std 版 Black Magic Probe 相当」を目指す。

（補足: 現状の rust-dap は CMSIS-DAP プローブであり、ホスト側の probe-rs/OpenOCD
が RSP↔SWD 変換を担っている。本提案はその変換をプローブ内に取り込む構想であり、
CMSIS-DAP 機能と排他ではなく、共存(別 USB インターフェース)も可能。）

---

## 1. レイヤ構成

```
USB-CDC (usbd-serial)                      … GDB とのシリアル通信路
   │  バイト列
RSP エンジン (gdbstub)                      … パケット framing / コマンド dispatch
   │  Target トレイト呼び出し(halt/step/read_mem/…)
Target: Cortex-M0+ デバッグ                 … DHCSR/DCRSR/FPB/DWT の操作
   │  ADIv5 メモリ/レジスタアクセス
ADIv5 (DP / MEM-AP)                         … DPIDR, CTRL/STAT, SELECT, CSW/TAR/DRW
   │  単発 SWD 転送(read/write, DP/AP, ack+retry)
rust_dap::DapTransport (SWD)               ← 既存資産をそのまま土台に
   │
PIO / bitbang ピン
        ├─ RP2040 フラッシュドライバ(bootrom 補助)  … GDB `load` / フラッシュ書換
        └─ RP2040 ターゲット定義(メモリマップ, multidrop, リセット)
```

**設計の要:** 最下層の「1回分の SWD 転送(DP/AP・R/W・ACK 判定・WAIT リトライ)」は
rust-dap の `DapTransport::swd_transfer` がすでに提供している。GDB デバッガは
その上に ADIv5 と Cortex-M デバッグ層を積むだけでよく、ビット叩き/PIO の再実装は不要。

---

## 2. 各層の設計

### 2.1 RSP エンジン: `gdbstub` クレート

- [`gdbstub`](https://crates.io/crates/gdbstub) は **no_std 対応**の Rust 製 RSP
  実装。パケットの framing・チェックサム・`+`/`-` ack・大半のコマンド解釈を担い、
  こちらは `Target` トレイト(と `SingleThreadBase` 等の IDT)を実装するだけ。
- 割り込み(Ctrl-C = `0x03`)、ソフト/ハードブレークポイント、メモリ R/W、
  レジスタ R/W、continue/step を trait メソッドとして受け取れる。
- 代替案: RSP を自前実装(BMP は C で自前)。パケット形式は単純なので可能だが、
  エッジケース(RLE 圧縮、`vCont`、`qSupported` ネゴシエーション、非ACKモード)を
  こなす gdbstub を使う方が堅実。**gdbstub 採用を推奨。**
- 注意: gdbstub のバージョンと no_std での features、ヒープ要否(`alloc` 不要な
  構成が可能か)を実装着手時に確認する。必要なら固定長バッファ運用にする。

### 2.2 ADIv5 (DP / MEM-AP)

ARM Debug Interface v5。`swd_transfer` を組み合わせて以下を実装:

- **DP**: 電源投入(CTRL/STAT の CDBGPWRUPREQ/CSYSPWRUPREQ)、エラークリア
  (ABORT)、`SELECT` による AP/バンク選択、`DPIDR` 読み出し。
- **MEM-AP**: `CSW`(転送サイズ/自動インクリメント)、`TAR`(アドレス)、
  `DRW`(データ)。32bit 単位の read/write と、TAR 自動インクリメントを使った
  ブロック転送。
- これらは `DapTransport` に依存する薄い構造体 `ArmDebugInterface<T: DapTransport>`
  として実装。probe-rs の `arm` 層が機能的リファレンス。

### 2.3 Cortex-M0+ (ARMv6-M) デバッグ

RP2040 のコアは Cortex-M0+。デバッグは標準の ARMv6-M デバッグ機構で、MEM-AP 経由で
以下のレジスタを叩く:

| 機能 | レジスタ | アドレス |
|---|---|---|
| Halt/Step/Run 制御 | DHCSR | 0xE000EDF0 |
| コアレジスタ選択 | DCRSR | 0xE000EDF4 |
| コアレジスタ入出力 | DCRDR | 0xE000EDF8 |
| デバッグ有効/ベクタキャッチ | DEMCR | 0xE000EDFC |
| ハードブレークポイント(FPB) | FP_CTRL/FP_COMP | 0xE0002000〜 |
| ウォッチポイント(DWT) | DWT | 0xE0001000〜 |
| リセット | AIRCR (SYSRESETREQ) | 0xE000ED0C |

- **halt/continue/step**: DHCSR に DBGKEY と C_DEBUGEN/C_HALT/C_STEP を書く。
- **レジスタ R/W**: DCRSR に番号+方向、DCRDR で値をやり取り。r0-r15, xPSR, MSP,
  PSP, CONTROL/PRIMASK。GDB の `g`/`G` パケットのレジスタ順にマップ。
- **ソフトウェアブレークポイント**: 命令を `BKPT` (0xBE00) に置換(RAM 上)。GDB が
  管理するので `Z0`/`z0` で当該アドレスの元命令を退避・復元。
- **ハードウェアブレークポイント**: FPB の 4 コンパレータ。フラッシュ上のコードに
  必須(命令を書き換えられないため)。`Z1`/`z1`。
- **ウォッチポイント**: DWT。`Z2`/`Z3`/`Z4`。
- **Cortex-M0+ の制約**: FPB は ARMv6-M 版(コンパレータ数が少ない、
  アドレス制約あり)、DWT の機能サブセットも限定。実装時に FP_CTRL/DWT_CTRL の
  NUM フィールドで実機の本数を読む。

### 2.4 RP2040 固有の制御(=「MCU が RP2040 を理解している」部分)

ここが本要件の核心。プローブ側ファームが把握すべき RP2040 の事情:

1. **SWD マルチドロップ**: RP2040 は 2 コアがそれぞれ独立の DP を持ち、
   さらに rescue DP がある。DP は SWD line reset 後に `TARGETSEL` 書き込みで選択:
   - Core0: `0x01002927`
   - Core1: `0x11002927`
   - Rescue DP: `0xf1002927`
   接続シーケンス(dormant→SWD 遷移、line reset、TARGETSEL、DPIDR 読み)を実装。
   rust-dap の `swj_sequence`/`swd_transfer` で表現可能。
2. **メモリマップ**: フラッシュ(XIP)0x10000000〜、SRAM 0x20000000〜、
   ROM 0x00000000〜、各ペリフェラル。GDB へのメモリマップ提供(`qXfer:memory-map`)。
3. **フラッシュ書き込み**(GDB `load` の実体): XIP フラッシュは直接書けないため、
   標準手法は **小さなフラッシュアルゴリズムを SRAM にロードして実行**する
   (OpenOCD/probe-rs 方式)。RP2040 の場合:
   - bootrom 関数(`connect_internal_flash`, `flash_exit_xip`,
     `flash_range_erase`, `flash_range_program`, `flash_flush_cache`,
     `flash_enter_cmd_xip`)をターゲットコアで呼ぶ、または
   - probe-rs の RP2040 flash algorithm(ELF/バイナリ)を SRAM に配置し、
     所定の ABI(バッファ/引数/完了ポーリング)で駆動する。
   まずは bootrom 関数呼び出し方式が実装量が小さい(ROM のエントリテーブルを
   `rom_func_table` から解決)。
4. **リセット**: DEMCR の VC_CORERESET でベクタキャッチ → AIRCR SYSRESETREQ、
   または WATCHDOG による reset。GDB の `R`/`vRun`/`extended-remote` の
   run/reset に対応。
5. **デュアルコア**: 初期は Core0 のみ対象(GDB シングルスレッド)。将来は
   2 コアを GDB のスレッド(`Hg`/`qfThreadInfo`/`vCont`)として提示。

RP2040 の知識は `trait Target`(または `struct Rp2040Target`)に閉じ込め、
メモリマップ・フラッシュ・multidrop・レジスタ数などをそこに集約。将来の
他チップ対応(STM32 等)は別 `Target` 実装として追加できる構造にする。

---

## 3. プローブ側の実行構造(RP2040 上)

現行 rust-dap の RTIC 構成を踏襲・拡張:

- **USBCTRL_IRQ**(高優先度): USB poll、CDC 受信バイトを RSP 入力キューへ、
  RSP 出力キューを CDC 送信へ。
- **rsp_task**(低優先度ソフトウェアタスク): 入力キューから RSP パケットを
  取り出し `gdbstub` に渡す。`Target` メソッドの中で SWD 転送が走る。
  長時間ブロッキング(フラッシュ書換・step 待ち)を割り込み外で行うのは、
  rust-dap で確立した「DAP 処理をタスクに逃がす」方針と同じ。
- SWD トランスポート(`DapTransport`)は rsp_task が排他所有(共有リソース化しない)。
- CMSIS-DAP と共存させる場合: USB 複合デバイスに CDC(GDB)+ Vendor(CMSIS-DAP)+
  CDC(UART ブリッジ)を並べる。ただし同一 SWD ピンを両者が使うため、
  同時利用は排他制御が必要 → 初版では **GDB サーバ専用ファーム**として分離する
  のが単純。

---

## 4. 依存クレートの評価

| 候補 | 用途 | 評価 |
|---|---|---|
| `gdbstub` | RSP | no_std 対応、Target trait 方式。**採用推奨**。alloc 要否を要確認 |
| `probe-rs` | ADIv5/flash 参照 | std 前提・大規模。**そのままは載らない**が、arm 層とフラッシュ ABI の設計・RP2040 flash algorithm の実装リファレンスとして参照 |
| rust-dap(本リポジトリ) | SWD 物理層 | `DapTransport` を土台に採用。ADIv5 を積む |
| Black Magic Probe (C) | 全体構成 | 機能的リファレンス。特に RP2040 対応・フラッシュ・multidrop の実装知見 |

ADIv5 と Cortex-M / RP2040 層は **自前実装**(no_std, ゼロアロケーション)。
probe-rs のロジックを Rust for embedded に写経する形になる。

---

## 5. マイルストーン(各段は実機検証可能)

1. **M1 — SWD/ADIv5 bring-up**: DapTransport 上に DP/MEM-AP を実装。
   接続・DPIDR 読み出し・SRAM/CHIP_ID 読み書き。
   *検証: 現在 probe-rs で読めている 0x40000000=0x20002927 を自前スタックで読む。*
2. **M2 — Cortex-M コア制御**: halt/run/step、コアレジスタ R/W(DHCSR/DCRSR)。
   *検証: 既知 RAM プログラムを halt して PC/レジスタを確認。*
3. **M3 — gdbstub 統合(RAM デバッグ)**: USB-CDC 上で `target extended-remote`
   接続、`info registers` / `x/` メモリダンプ / `continue` / `stepi` /
   ソフトブレークポイント(RAM)。
   *検証: SRAM で動くターゲットプログラムを実機 GDB でステップ実行。*
4. **M4 — ハードブレーク/ウォッチ**: FPB / DWT。フラッシュ常駐コードの
   ブレークに対応。
5. **M5 — フラッシュ書換**: bootrom 補助でフラッシュアルゴリズムを駆動、
   GDB `load` に対応。
   *検証: `load` で .elf を焼いて `continue`。*
6. **M6 — デュアルコア/リセット**: multidrop で Core0/Core1 を GDB スレッド化、
   ベクタキャッチリセット。

M1〜M3 が MVP(RAM 上のプログラムをフル GDB デバッグできる)。
M5 到達で実用的なフラッシュデバッガになる。

---

## 6. リポジトリ構成案

- 新クレート `arm-debug`(no_std, HAL 非依存): ADIv5 + Cortex-M デバッグ。
  `DapTransport` にのみ依存。→ rust-dap と同じく再利用可能なコアに。
- 新クレート `rp2040-target`: RP2040 固有(メモリマップ, multidrop, bootrom
  フラッシュ)。`arm-debug` に依存。
- 新ボードファーム `boards/rpi_pico_gdb`(または既存 rpi_pico に feature 追加):
  gdbstub + USB-CDC + `arm-debug` + `rp2040-target` を RTIC で結線。
- これらは rust-dap 本体とは独立(DapTransport でのみ結合)なので、
  現行 CMSIS-DAP ファームに影響を与えず追加できる。

---

## 7. オープンな論点(要判断)

1. **gdbstub の alloc 依存**: no_std かつ no-alloc で目的コマンドを賄えるか。
   賄えない場合、`embedded-alloc` を入れるか RSP を自前実装するか。
2. **フラッシュアルゴリズムの供給元**: bootrom 関数呼び出し方式(実装小)と
   probe-rs flash algorithm 流用(汎用)のどちらを M5 の初手にするか。
3. **CMSIS-DAP との同居**: 初版は GDB 専用ファームとして分離でよいか。
4. **プローブとターゲットが同一 RP2040 である前提**: プローブ側の SWD クロックや
   ピン割当(現行 GPIO2=SWCLK, GPIO3=SWDIO, GPIO4=RESET)を踏襲でよいか。
5. **スコープ**: まず M1〜M3(RAM デバッグ MVP)を確定ゴールにしてよいか。

---

## 8. Baochip / Dabao / Xous への移植性(調査結果 2026-07-14)

将来ターゲットとして挙がった Baochip(bao1x)/ Dabao ボード / Xous について、
一次情報(各リポジトリ・RTL・bunnie のブログ・Xous book)を調査した。要点:

### 前提: これらは RISC-V であり ARM ではない
- **Xous**: Rust 製マイクロカーネル OS。Precursor 上では Xilinx FPGA の
  **VexRiscv(RV32IMAC + Sv32 MMU)** ソフトコア。
- **Baochip (bao1x)**: bunnie の Precursor 後継、22nm カスタム SoC。
  メイン CPU は **VexRiscv RV32IMAC + Sv39**(約 350MHz)、USB2.0 HS デバイス
  内蔵、RRAM 4MiB。Dabao はその評価ボード(USB-C + 20 GPIO、**JTAG/SWD
  コネクタなし**)。

本デバッガの現行スタックは全て ARM 専用(SWD/JTAG → **ADIv5** → Cortex-M)。
RISC-V は ADIv5 も Cortex-M デバッグも持たないため、下記2方向で評価する。

### (A) プローブファーム自体をこれらの上で動かす — **有望・中程度**
- Baochip / Dabao は bare-metal Rust ターゲット
  (`riscv32imac-unknown-none-elf`)、USB-HS デバイス、Rust ドライバが
  xous-core と samblenny/baochip-sdk に存在。
- 必要作業: `bitbang.rs` の **`BidirPin` を bao1x GPIO に実装**、`Delay` 実装、
  USB クラス配線。**コアの `cmsis_dap`/`dispatcher`/`transport`/`arm-debug` は
  HAL 非依存でそのままコンパイル可能**。
- 障害はアーキテクチャではなく HAL/PAC の未成熟(crates.io に `bao1x-hal` は
  無く、ドライバは xous-core 等の in-tree)と USB クラス/PHY 周りのグルー。
- 成果物: bao1x/Dabao をホストとする CMSIS-DAP/SWD プローブ(=RP2040/xiao 版と
  同じ位置づけ。ARM ターゲットをデバッグする)。**RP2040 の次の移植先として妥当。**

### (B) これらを *ターゲット* としてデバッグする — **不向き・大規模な新規実装**
- CPU デバッグは VexRiscv の **独自 DebugPlugin バス**(`GenCramSoC.scala` で
  `hardwareBreakpointCount = 4` の旧 DebugPlugin を `fromJtag()` で接続)。
  **標準 RISC-V Debug Spec 0.13/1.0 の DM/DMI ではなく**、SpinalHDL パッチ版
  OpenOCD しか話せない。ADIv5 は当然使えない。
- Dabao には CPU デバッグ用 JTAG コネクタが出ていない(オンパッケージ JTAG は
  DFT/バウンダリスキャン用)。物理的にプローブを繋ぐ先が無い。
- **既存の代替が優秀**: Xous カーネルは既に Rust の **`gdbstub` クレート**を使った
  RISC-V マルチスレッド GDB サーバを内蔵(`kernel/src/debug/gdb.rs`、
  precursor と **bao1x 両方の backend** あり)。Precursor は USB-Wishbone
  ブリッジで CPU halt も可能。rust-dap で置き換える価値は乏しい。

### 再利用可能性の内訳((B)方向)
| 本リポジトリの層 | RISC-V DM ターゲットでの再利用 |
|---|---|
| `bitbang.rs` の JTAG ビット/TAP エンジン(`JtagBits`) | **再利用可**(電気層の IR/DR シフト・TMS 状態機械)。唯一移植可能な部分 |
| `bitbang.rs` SWD(`SwdBits`) | 不可(ARM 専用ワイヤプロトコル) |
| `arm-debug`(ADIv5 DP/AP/MEM-AP) | **不可** — RISC-V Debug Module ドライバ(DTM→DMI→DM レジスタ、abstract command、program buffer)へ置換。VexRiscv 独自バスならさらに専用ドライバ |
| Cortex-M コアデバッグ層(M2 以降) | 不可 — RISC-V レジスタファイル/CSR・`dcsr/dpc`・trigger へ置換 |
| gdbstub/RSP 層(本計画で新規) | 発想は共通(Xous も同じ `gdbstub` クレート採用) |

### 結論
- **最有望**: rust-dap を **Baochip/Dabao 上で動かす**(方向A)。bao1x は有能な
  USB-HS RISC-V MCU で、コードはアーキテクチャ的に移植可能。コストは HAL/USB
  スタックの成熟度であってデバッグ設計ではない。RP2040 の次の移植先候補。
- **不向き/冗長**: これらを RISC-V ターゲットとしてデバッグすること。エコシステムが
  既に Rust `gdbstub` サーバと(Precursor では)USB-Wishbone CPU-halt を持つ。
- **もし RISC-V ターゲット対応を将来やるなら**: 標準 RISC-V コア向けに
  RISC-V Debug Module ドライバを新規実装する形にし、`JtagBits` を電気層として
  再利用、`gdbstub` を RSP 層に据える。ARM の `arm-debug` と対になる
  `riscv-debug` クレートを別途起こすのが構造的に自然。

### 一次情報
- bunnie's blog (Baochip-1x): https://www.bunniestudios.com/blog/2026/baochip-1x-a-mostly-open-22nm-soc-for-high-assurance-applications/
- Coder's Guide to the Baochip 1x: https://baochip.github.io/baochip-1x/
- Baochip-1x RTL(VexRiscv 設定): https://github.com/baochip/baochip-1x — `VexRiscv/GenCramSoC.scala`
- Dabao ボード: https://github.com/baochip/dabao , https://www.crowdsupply.com/baochip/dabao
- samblenny/baochip-sdk(bare-metal Rust): https://github.com/samblenny/baochip-sdk
- xous-core(カーネル gdbstub): https://github.com/betrusted-io/xous-core — `kernel/src/debug/gdb.rs`, `README-baochip.md`
- Xous book(デバッグ): https://betrusted.io/xous-book/ch03-04-debugging-programs.html
- RISC-V External Debug Support 0.13: https://riscv.org/wp-content/uploads/2024/12/riscv-debug-release.pdf

---

## 9. Xous(Baochip 公式 OS)の USB 詳細と移植への影響(調査結果 2026-07-14)

Baochip の公式サポートが Xous であることを踏まえ、Xous の USB スタックを
xous-core のソースで確認した(全て betrusted-io/xous-core @ main)。

### USB スタック構成
- **Rust `usb-device` クレート(betrusted-io フォーク)** + `usbd-serial`(CDC-ACM)
  + `usbd_scsi`/`usbd_mass_storage`(in-tree)+ HID。rust-dap と同じ基盤。
- **Precursor**: 単一デバイスに対する**排他的 "View"**として役割を切替
  (`services/usb-device-xous/src/main_hw.rs`)。同時併存の複合デバイスではない:
  - `FidoWithKbd`(HID キーボード + U2F)/ `MassStorage` / **`Serial`(CDC-ACM)** / `HIDv2`
  - VID `0x1209` / PID `0x3613`
- **bao1x**: 所有者が2つ。**bootloader (boot1)** が UF2 用 Mass Storage と
  CDC-ACM(IAD 付き, VID `0x1d50`/PID `0x6196`)を提供。**稼働システム**は
  「USB 経由シリアル=デバッグコンソール」を提供(`README-baochip.md`)。

### GDB/デバッグ経路
- **カーネル gdbstub は USB-CDC ではなく物理 UART 専用**。
  `kernel/src/platform/{bao1x,precursor}/gdbuart.rs` はメモリマップ UART
  レジスタドライバ(`0xffcc_0000` + IRQ)。USB-CDC シリアルは対話コンソールで
  gdbstub とは別系統。Dabao では PB14/PB13 の 1Mbaud UART。
- UF2 の `BAOCHIP` Mass Storage ボリュームは **bootloader モード**(PROG ボタン)
  側で、稼働 OS の USB 構成とは別 owner・別モード。

### rust-dap プローブを Xous に乗せられるか
- **Vendor/bulk(CMSIS-DAP v2)や任意 CDC インターフェースはアプリから追加不可**。
  デバイスディスクリプタはシステムサービス `usb-device-xous` が固定所有。アプリは
  IPC で**固定 enum の役割(Debug/Keyboard/FidoU2f/MassStorage/Serial/HIDv2)を
  要求できるだけ**。追加にはサービス自体のパッチが必要。役割は排他なので、プローブ
  用インターフェースを出すとコンソールを排除する。
- **唯一の抜け道は HIDv2**: アプリが自前 HID レポートディスクリプタ(64B in/out
  固定)を `HIDSetDescriptor` で登録できる(`services/usb-device-xous/src/hid.rs`,
  `apps/hidv2`)。**CMSIS-DAP v1 はまさに 64B in/out の HID デバイス**なので、
  原理的にはシステム改造なしに Xous アプリとして DAP v1 相当を実装可能
  (他の USB 役割と排他、tree 内前例なし=未検証)。

### 結論(方向A の具体像)
- **公式 Xous に乗せる場合**、rust-dap を「ファームウェアとして移植」はできない。
  選択肢は (i) HIDv2 で **CMSIS-DAP v1 相当を Xous アプリ**として実装
  (コマンドロジックは rust-dap の `dispatcher`/`cmsis_dap` を流用、USB は
  Xous HID サービス経由)、または (ii) `usb-device-xous` に Vendor/追加 CDC
  インターフェースを足す**システムサービス改造**(CMSIS-DAP v2 / GDB-CDC 用)。
- **bare-metal(samblenny/baochip-sdk)なら**この制約は無く、rust-dap の USB 層を
  そのまま(bao1x USB PHY 上に)載せられる。「公式 Xous 重視」と「実装の素直さ」の
  トレードオフ。

### 一次情報
- `services/usb-device-xous/{Cargo.toml, src/main_hw.rs, src/api.rs, src/hid.rs}`, `apps/hidv2/`
- `libs/bao1x-hal/`, `libs/mass-storage/`, root `Cargo.toml` (`[patch.crates-io.usb-device]`)
- `kernel/src/debug/gdb.rs`, `kernel/src/platform/{bao1x,precursor}/gdbuart.rs`
- `bao1x-boot/boot1/src/platform/bao1x/usb/mod.rs`, `README-baochip.md`
- https://github.com/betrusted-io/xous-core/tree/main/
