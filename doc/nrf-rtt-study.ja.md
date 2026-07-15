# nRF52/nRF54 ターゲット対応 & SEGGER RTT 対応 — 実現可能性・設計検討

作成日: 2026-07-15
対象コード: `arm-debug/src/lib.rs`(ADIv5 + Cortex-M デバッグ層)、
`boards/rpi_pico/src/bin/gdb_server.rs`(gdbstub ベース RSP サーバ)

本書は standalone GDB デバッガ(`doc/gdb-debugger-proposal.ja.md` 参照)の
次の 2 つの拡張方向についての検討結果をまとめる。

1. Nordic nRF52 / nRF54 シリーズをデバッグターゲットとして追加する
2. SEGGER RTT 相当のターゲット I/O(J-Link RTT 相当)をプローブに実装する

---

# 第 1 部: nRF52 / nRF54 ターゲット対応

## 1. 現状整理 — 何がアーキテクチャ共通で、何が RP2040 固有か

現在の実装を「そのまま使える層」「変更が要る層」に分解すると:

### そのまま使える(アーキテクチャ共通)

| 層 | 根拠 |
|---|---|
| SWD ワイヤプロトコル(`DapTransport`、line reset、WAIT リトライ、sticky error クリア) | ADIv5 共通 |
| DP レジスタ(DPIDR/ABORT/CTRL_STAT/SELECT/RDBUFF)と power-up シーケンス | ADIv5 共通(ADIv6 の nRF54H は除く。§4) |
| MEM-AP CSW/TAR/DRW、TAR 1KiB オートインクリメント、`read_mem`/`write_mem` | MEM-AP 共通 |
| コアデバッグ: DHCSR/DCRSR/DCRDR/DEMCR/DFSR/AIRCR(`cortex_m` モジュール全体) | **ARMv6-M / v7-M / v8-M でアドレス・ビット割当てとも共通**(0xE000_EDF0〜)。halt/resume/step/レジスタ R/W/`call_function`/vector catch はそのまま動く |
| DWT ウォッチポイント(v6-M/v7-M。v8-M は差分あり、§3.4) | ほぼ共通 |
| gdbstub 層(`RpTarget` の骨格、USB-CDC 接続、セッション管理) | ターゲット非依存 |

### 変更が要る(ターゲット固有)

| 項目 | RP2040 | nRF52 | nRF54L |
|---|---|---|---|
| SWD 接続シーケンス | multidrop(dormant→SWD→TARGETSEL) | **通常 SW-DP: line reset + JTAG→SWD(0xE79E)+ line reset** | DPv2(DPIDR 0x6BA02477)だが single-drop 接続で可 |
| コア数/コア選択 | 2 コア = 2 DP(TARGETSEL 切替) | 1 コア | M33 1 + RISC-V VPR(GDB 対象は M33 のみ) |
| AP 構成 | AP #0 のみ | AHB-AP #0 + **CTRL-AP #1** | MEM-AP #0 + **CTRL-AP #2** |
| ブレークポイントユニット | FPB v1(ARMv6-M) | FPB v1(Cortex-M4、6 comparator) | **FPB rev2 相当(ARMv8-M)** — エンコーディング変更 |
| フラッシュ書込 | bootrom 関数呼出(`call_function`) | **NVMC レジスタ叩くだけ**(はるかに簡単) | **RRAMC — 消去不要の直接書込** |
| リセット | AIRCR.SYSRESETREQ | SYSRESETREQ + CTRL-AP.RESET(ハード相当) | 同左 |
| 保護解除/全消去 | rescue DP | CTRL-AP.ERASEALL | CTRL-AP.ERASEALL |

## 2. nRF52 シリーズ(Cortex-M4、例: nRF52840)

### 2.1 SWD 接続

nRF52 の DP は SWJ-DP(DPIDR 0x2BA01477 = DP アーキテクチャ v1)。
multidrop ではないので TARGETSEL・dormant シーケンスは不要かつ非対応。
必要なシーケンス:

1. line reset(≥50 クロック High)
2. JTAG→SWD 切替シーケンス `0xE79E`(16bit, LSB first)
3. line reset + ≥2 idle クロック
4. DPIDR 読み出し

`arm-debug` への追加: `connect_multidrop()` と対になる
`connect_swd()`(0xE79E 版)。`swd_line_reset()` / `power_up()` /
sticky クリアはそのまま再利用できる。dormant シーケンス送出は v1 DP には
未定義動作に近い(実際は無視されることが多い)ので、送らない別経路にする。

参考: OpenOCD/J-Link のログでは nRF52840 は
`SWD DPIDR 0x2ba01477`, `Cortex-M4 r0p1`, `6 breakpoints, 4 watchpoints`
と検出される([SEGGER forum](https://forum.segger.com/thread/9859-solved-new-version-8-48-cannot-flash-nrf52840-chips/) 等のセッションログ)。

### 2.2 FPB — v7-M で何が変わるか

現行 `fpb` モジュールは ARMv6-M FPB v1 前提。v7-M(Cortex-M4)の FPB も
**rev1** であり、コンパレータのエンコーディング
(`REPLACE[31:30] | COMP[28:2] | ENABLE`、コード領域 < 0x2000_0000 限定)は
**同一**。つまり nRF52 では `fpb_comp_value()` はそのまま正しい。

ただし 2 点修正が要る:

1. **NUM_CODE の読み方**: v7-M では `FP_CTRL.NUM_CODE` は
   `bits[14:12]:bits[7:4]` の 7bit(DDI0403 C1.11)。現行の
   `(ctrl >> 4) & 0xF` は nRF52(6 個)では偶然正しいが、一般には
   `((ctrl >> 8) & 0x70) | ((ctrl >> 4) & 0xF)` に直すべき。
2. **REV フィールドの確認**: `FP_CTRL[31:28]` が REV。REV=0 が rev1
   (M0+/M3/M4)、REV=1 が rev2(M7、および ARMv8-M の FPB)。
   **rev2 では FP_COMP が `BPADDR[31:1] | BE[0]` になり、REPLACE
   フィールドが消えて全アドレス空間にブレーク可能**になる
   ([Doulos FPB app note](https://www.doulos.com/media/1188/using_cortex-m3_fpb.pdf)、
   [ARMv7-M ARM DDI0403](https://developer.arm.com/documentation/ddi0403/d/))。

→ 実装方針: 接続時に `FP_CTRL.REV` を 1 回読んでキャッシュし、
`fpb_comp_value(addr, rev)` を rev で分岐させる。トレイト分岐は不要、
**ランタイム判別で v1/v2 両対応が 20 行程度**で済む。

### 2.3 DWT

v7-M の DWT はコンパレータのレジスタ配置(COMP/MASK/FUNCTION、0x10 刻み)、
NUMCOMP[31:28]、FUNCTION の 0b0101/0110/0111 エンコーディングとも現行実装と
互換。DEMCR bit24 は v7-M では TRCENA と呼ばれるが同じビット。
**nRF52 では DWT コードは無変更で動く見込み。**

### 2.4 コアレジスタ(FPU)

DCRSR セレクタは r0-r15/xPSR/MSP/PSP まで共通。nRF52840 は FPv4-SP を持ち、
FPU レジスタは DCRSR セレクタ **33 = FPSCR、64..95 = S0..S31**、
セレクタ 20 = CONTROL/FAULTMASK/BASEPRI/PRIMASK(パック)。
gdbstub 側は現在 `Armv4t`(コアレジスタのみ)なので、**初期対応では FPU
レジスタなしで一切問題ない**(GDB は見えないレジスタを触らない)。
FPU 対応は `gdbstub_arch` のターゲット記述差し替え+read/write_registers の
拡張で後付け可能(優先度低)。

### 2.5 NVMC フラッシュ書込 — RP2040 より大幅に簡単

RP2040 では XIP 解除+bootrom 関数のリモート呼出(`call_function`、
トランポリン、スタック確保)が必要だったが、nRF52 の NVMC は
**MEM-AP からのレジスタ書込だけで完結**する。コアは halt したままでよく、
ターゲット RAM も消費しない。

NVMC ベース 0x4001_E000
([nRF52840 PS: NVMC](https://docs.nordicsemi.com/bundle/ps_nrf52840/page/nvmc.html))。

| レジスタ | オフセット | 内容 |
|---|---|---|
| READY | 0x400 | bit0=1 で NVMC ready |
| READYNEXT | 0x408 | 次の書込を受付可能 |
| CONFIG | 0x504 | WEN: 0=Ren(読取専用) / 1=Wen(書込許可) / 2=Een(消去許可) |
| ERASEPAGE | 0x508 | ページ先頭アドレスを書くとそのページ(4 KiB)消去 |
| ERASEALL | 0x50C | 1 を書くと全消去(フラッシュ+UICR) |
| ERASEUICR | 0x514 | UICR 消去 |

手順(いずれも `read_word`/`write_word` のみで実装可):

```text
ページ消去: CONFIG=2 → ERASEPAGE=page_addr → READY==1 をポーリング(〜85ms) → CONFIG=0
ワード書込: CONFIG=1 → フラッシュアドレスに 32bit ワードを直接書く(MEM-AP 経由、
            TAR オートインクリメントのブロック書込がそのまま使える)
            → READY ポーリング(〜41µs/word) → CONFIG=0
```

制約: 書込は 32bit ワード単位・ワードアライメント、1→0 方向のみ
(RP2040 と同じ「erase 済みセクタへの追記」ロジック=`erased` ビットマップが流用できる)。
nRF52840 は 1 MiB / 4 KiB ページ × 256。

`gdb_server.rs` の `flash_write()` は「erase 済み管理+ページ単位処理」の
骨格が既にあるので、`rom_call` 部分を NVMC レジスタ操作に置換した
nRF52 版を書くだけ。**XIP モード出入り(`flash_enter`/`flash_exit`)の
概念自体が不要**(nRF のフラッシュは常時リード可能)。

### 2.6 CTRL-AP(AP #1)— 保護解除・リカバリ・リセット

nRF52 は AP #1 に Nordic 独自の CTRL-AP を持つ(IDR 0x02880000)。
APPROTECT 有効時に AHB-AP が遮断されても CTRL-AP は常にアクセス可能
([Luca Bruno: nRF52 DAP protection](https://www.lucabruno.net/posts/2019-12-26/nrf52-ap-protection/)、
[Nordic blog: improved APPROTECT](https://devzone.nordicsemi.com/nordic/nordic-blog/b/blog/posts/working-with-the-nrf52-series-improved-approtect))。

| レジスタ | オフセット | 内容 |
|---|---|---|
| RESET | 0x000 | 1 を書くとリセット保持、0 で解放(ソフトリセット、ピンリセット相当) |
| ERASEALL | 0x004 | 1 を書くと全消去開始(フラッシュ+UICR+RAM)。保護解除の唯一の手段 |
| ERASEALLSTATUS | 0x008 | 0=Ready / 1=Busy |
| APPROTECTSTATUS | 0x00C | bit0: 1=保護無効(アクセス可)、0=保護有効 |
| IDR | 0x0FC | 0x0288_0000 |

リカバリシーケンス(`monitor erase_all` として実装する):

```text
1. AP#1 選択 → APPROTECTSTATUS 読取(保護状態の報告)
2. ERASEALL=1 → ERASEALLSTATUS==Ready をポーリング(全消去は数百 ms)
3. RESET=1 → RESET=0(または pin reset)→ 再接続
```

**必要な arm-debug 変更**: 現行 `select_ap_bank()` は APSEL=0 固定。
`SELECT[31:24]=APSEL` を引数化し、`ap_read_apsel(apsel, addr)` /
`ap_write_apsel(...)` を追加する(数十行)。CSW キャッシュは AP を跨いだら
無効化する。

注意(新 APPROTECT): nRF52840 の新しいリビジョン(および nRF52833 等)では
UICR.APPROTECT=HwDisabled(0x5A)に加えてファームウェアが起動時に
APPROTECT.DISABLE を書かないとデバッグアクセスが有効にならない。
「フラッシュを消したのに繋がらない」時はこれを疑う旨をドキュメント化する。

### 2.7 リセット

- 通常デバッグ: 既存の AIRCR.SYSRESETREQ 経路がそのまま使える
  (`target_reset()` の VC_CORERESET も v7-M 共通)。
- nRF52 のリセットピンは P0.18(デフォルト。UICR/PSELRESET で構成)。
  プローブの GPIO4(RESET)を繋げばハードリセットも可能だが、
  CTRL-AP.RESET が同等の効果を持つため必須ではない。
- ERASEALL 後は pin/power reset まで一時的にデバッグ可能、という
  挙動があるので `monitor erase_all` の後は CTRL-AP.RESET → 再接続を
  自動で行うのがよい。

## 3. nRF54 シリーズ(公開情報ベース)

### 3.1 nRF54L(nRF54L15/L10/L05)— Cortex-M33 (ARMv8-M)

公開データシート
([nRF54L15 datasheet](https://docs.nordicsemi.com/bundle/ps_nrf54L15/page/debug.html))より:

- CPU: Cortex-M33 @128 MHz + RISC-V コプロセッサ(FLPR/VPR)。
- DP: **標準 CoreSight SW-DP、DPIDR 0x6BA02477(= DP アーキテクチャ v2、
  ADIv5.2)**。ADIv6 ではない。DPv2 なので TARGETSEL レジスタは存在するが
  single-drop 構成であり、接続は nRF52 と同じ単純シーケンスで可
  (DPv2 は dormant もサポートするので、既存の dormant→SWD 経路も使える
  はず — RP2040 と同じ `dormant_to_swd()` が流用できる可能性が高い)。
- AP 構成: **MEM-AP #0(RRAM/ペリフェラルへのメモリアクセス)、
  CTRL-AP #2(リカバリ・ERASEALL)**
  ([nRF54L15 CTRL-AP](https://docs.nordicsemi.com/bundle/ps_nrf54L15/page/ctrl-ap.html))。
  CTRL-AP のレジスタセット(ERASEALL/ERASEALLSTATUS/APPROTECT 系)は
  nRF52/nRF5340 系の発展形で、オフセットはデータシートで要確認
  (nRF5340 以降 APPROTECT.DISABLE の書込レジスタが CTRL-AP に増えている)。

**RRAMC(0x5004_B000 secure / 0x4004_B000 non-secure)**
([RRAMC](https://docs.nordicsemi.com/bundle/ps_nrf54L15/page/rramc.html)、
[OpenOCD nRF54 NVM driver patch](https://www.mail-archive.com/openocd-devel@lists.sourceforge.net/msg17300.html)):

- RRAM(抵抗変化型メモリ)は **消去不要・直接上書き可能**。
  ERASEPAGE に相当するものはなく(ERASE.ERASEALL のみ)、
  「erase 済みビットマップ」ロジックごと不要になる。
- 書込手順(OpenOCD ドライバより): `CONFIG.WEN=1` → 対象アドレスに
  ワード書込(write buffer に溜まる)→ `TASKS_COMMITWRITEBUF`(offset 0x008)
  に 1 を書いてコミット → `READY`/`READYNEXT` ポーリング → `CONFIG=0`。
  つまり **nRF52 NVMC とほぼ同じ形のレジスタ駆動**で、フラッシュドライバの
  抽象(§5 の `FlashAlgo`)に素直に収まる。
- TrustZone: リセット後は RRAM/RAM とも Secure 属性。RRAMC の
  CONFIG は Secure アクセスでのみ書ける。デバッガ側の対処は
  **MEM-AP CSW.PROT の HNONSEC ビットを 0(Secure アクセス)にする**こと。
  現行 `CSW_DEFAULT = 0x2300_0000 | ...` の PROT ビット割当てが
  nRF54L の AHB5 でどう解釈されるかは実機確認項目(M-nRF5)。

**ARMv8-M のデバッグ差分**(nRF54L/nRF5340 共通):

- DHCSR/DCRSR/DCRDR/DEMCR/DFSR/AIRCR: **アドレス・主要ビットとも共通**。
  halt/step/レジスタアクセス/vector catch は無変更で動く見込み。
  DHCSR に S_SDE(Secure debug enabled)等の状態ビットが増えるが読み飛ばせる。
- FPB: v8-M の FPB は **rev2 エンコーディング**(FP_COMP =
  `BPADDR[31:1] | BE`、全アドレス空間対応)。§2.2 の REV ランタイム判別で
  同時に対応できる。
- DWT: v8-M では **MASKn レジスタが廃止**され、FUNCTION の
  MATCH/DATAVSIZE フィールド構成に変わる(範囲ウォッチはコンパレータの
  ペア使用)。現行の `watchpoint_set`(COMP+MASK+FUNCTION)は v8-M では
  そのまま動かないため、ここは **DWT flavor の分岐が必要**(§5)。
  最小対応: DATAVSIZE(1/2/4 バイト)+ MATCH=アドレス一致のみサポートし、
  `len>4` の範囲ウォッチは拒否する。
- DCRSR: Secure/Non-secure バンクの MSP_NS/PSP_S 等セレクタが増えるが、
  初期対応では不要。

### 3.2 nRF5340(中間ステップとして有用)

nRF54 の前に、入手性の良い nRF5340(dual Cortex-M33)を M33/TrustZone の
検証台にできる。AP 構成は
**#0=app AHB-AP、#1=net AHB-AP、#2=app CTRL-AP(IDR 0x12880000)、#3=net CTRL-AP**
([nRF5340 CTRL-AP](https://infocenter.nordicsemi.com/topic/ps_nrf5340/ctrl-ap.html)、
[Nordic blog: nRF5340 debugger access](https://devzone.nordicsemi.com/nordic/nordic-blog/b/blog/posts/allowing-debugger-access-to-an-nrf5340))。
2 コアだが RP2040 と違い **1 つの DP に複数 AP**(multidrop ではない)なので、
「コア選択 = APSEL 切替」となり、`select_core()` の抽象化(§5)の良い実例になる。
フラッシュは NVMC(app: 0x5003_9000 / net: 0x4108_0000)で nRF52 とほぼ同じ。

### 3.3 nRF54H(nRF54H20)— 当面スコープ外を推奨

公開情報([nRF54H20 debugging, NCS docs](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/app_dev/device_guides/nrf54h/ug_nrf54h20_debugging.html))
では AHB-AP(ID 3)/CTRL-AP(ID 4)/APB-AP(ID 5)を持つ CoreSight
サブシステム構成で、**ADIv6 世代**。ADIv6 対応には:

- DP SELECT の意味が変わる(APSEL 8bit → AP はアドレス空間にマップ、
  SELECT.ADDR[31:4] + SELECT1 で 64bit AP アドレス)
- DPIDR1/BASEPTR0/1 による AP ROM テーブル探索
- AP レジスタオフセットが 0xD00 台(CSW=0xD00, TAR=0xD04, DRW=0xD0C)

と DP/AP 層の改修が大きい。さらに Secure Domain(IronSide)経由の
デバッグ許可など製品固有の手続きがある。**M-nRF マイルストーンからは
切り離し、「将来課題」として DP 層に version 分岐の余地だけ残す**のが現実的。

### 3.4 差分まとめ表

| 機能 | nRF52840 | nRF5340/nRF54L | 対応箇所 |
|---|---|---|---|
| 接続 | line reset + 0xE79E | 同左(DPv2 なので dormant 経路も可) | `connect_swd()` 新設 |
| APSEL | AHB-AP#0, CTRL-AP#1 | #0/#2(5340 は #0..#3) | `select_ap_bank` の APSEL 引数化 |
| FPB | rev1(現行と同一) | rev2(BPADDR エンコーディング) | REV ランタイム判別 |
| DWT | 現行と同一 | MASK 廃止 → FUNCTION 再設計 | DWT flavor 分岐 |
| コアデバッグ | 共通 | 共通(+Secure 状態ビット) | 変更なし |
| フラッシュ | NVMC レジスタ | NVMC / RRAMC レジスタ | `FlashAlgo` 実装追加 |
| TrustZone | なし | CSW.PROT(HNONSEC)配慮 | CSW 値のターゲット化 |

## 4. arm-debug の抽象化設計

方針: **「動的ディスパッチの Target トレイト」を firmware バイナリ内に 1 つ
持つ**のではなく、まずは以下の 3 段階で切る。

### 4.1 arm-debug 内(機構の一般化 — トレイト不要)

1. `connect_swd()`(非 multidrop 接続)追加。
2. `select_ap_bank` → APSEL 対応(`ap_read/write` に apsel 引数版を追加、
   既存 API は apsel=0 のラッパとして維持)。
3. FPB REV(v1/v2)ランタイム判別: `FP_CTRL[31:28]` 読取 →
   `fpb_comp_value(addr, rev)`。NUM_CODE 読取の 7bit 化。
4. DWT flavor: `DWT_CTRL` だけでは v8-M 判別ができないため、
   CPUID(0xE000_ED00)の Architecture フィールドで v6/v7-M と v8-M を
   判別してエンコーディングを切替。
5. CSW 既定値の設定可能化(TrustZone: HNONSEC/Prot ビット)。

### 4.2 Target トレイト(gdb_server 側、コンパイル時選択)

ファームウェアはターゲットごとにビルドする(RP2040 プローブ 1 台 =
1 ターゲットファミリ)前提で、**ジェネリクスによる静的ディスパッチ**にする:

```rust
pub trait TargetFamily {
    const N_CORES: usize;
    /// 接続シーケンス(multidrop or plain)+ 全コア halt
    fn connect(&mut self, arm: &mut ArmDebug<T>) -> Result<u32, ArmError>;
    /// コア選択(TARGETSEL 切替 or APSEL 切替 or 単一コアで no-op)
    fn select_core(&mut self, arm: &mut ArmDebug<T>, core: usize) -> Result<(), ArmError>;
    /// メモリマップ(フラッシュ base/size/sector、RAM 範囲=RTT スキャン範囲兼用)
    fn memory_map(&self) -> &'static [MemRegion];
    /// フラッシュ書込(erase 管理込み。RP2040=bootrom call、nRF52=NVMC、nRF54L=RRAMC)
    fn flash_write(&mut self, arm: &mut ArmDebug<T>, addr: u32, data: &[u8]) -> Result<(), ArmError>;
    fn flash_end(&mut self, arm: &mut ArmDebug<T>);   // RP2040 の XIP 復帰、nRF は no-op
    fn reset(&mut self, arm: &mut ArmDebug<T>, halt: bool) -> Result<u32, ArmError>;
    /// monitor コマンドのファミリ固有部(erase_all 等)
    fn monitor(&mut self, arm: &mut ArmDebug<T>, cmd: &[u8], out: &mut ConsoleOutput) -> bool;
}
```

`RpTarget` を `GdbTarget<F: TargetFamily>` に一般化し、RP2040 固有部
(`rom_flash_fns`/`flash_enter`/`flash_exit`/`CORE_TARGETSEL`/XIP 判定)を
`Rp2040Family` に移す。diag 窓・USB 層・gdbstub 実装は共通のまま。

### 4.3 マイルストーン(nRF)

| ID | 内容 | 検証 | 規模感 |
|---|---|---|---|
| **M-nRF1** | `connect_swd()`(0xE79E)+ APSEL 対応。nRF52840 DK に対し DPIDR/halt/レジスタ/RAM R/W | GDB attach、`info registers`、RAM `x`/`set` | 小(arm-debug 数十行+接続切替) |
| **M-nRF2** | FPB NUM_CODE 7bit 化 + REV 判別(rev1 実機確認)。BKPT/FPB/DWT ブレーク・ウォッチ動作 | `break`/`hbreak`/`watch` E2E | 小 |
| **M-nRF3** | NVMC フラッシュドライバ + TargetFamily トレイト導入(RP2040 を回帰させない) | GDB `load` で Zephyr/nRF SDK ELF 書込→実行 | 中(リファクタ含む) |
| **M-nRF4** | CTRL-AP: `monitor erase_all` / `monitor reset`(CTRL-AP.RESET)、APPROTECT 状態報告 | 保護済みチップのリカバリ | 小 |
| **M-nRF5** | ARMv8-M 対応: FPB rev2 / DWT v8-M flavor / CSW Secure 設定。nRF5340 or nRF54L15 DK で検証。RRAMC 書込(54L) | M33 での full E2E + `load` | 中〜大 |
| **M-nRF6** | (将来)ADIv6 = nRF54H。DP SELECT/BASEPTR 対応 | — | 大、当面着手しない |

推奨着手順: **M-nRF1 → M-nRF2 は nRF52840 DK 1 枚で 1〜2 日規模**。
コアデバッグ層が無変更で動くことを最初に実証すると、以降の工数見積りの
不確実性が大きく下がる。

---

# 第 2 部: SEGGER RTT 対応

## 5. RTT の仕組み

RTT はターゲット RAM 上のリングバッファをデバッグプローブが
**MEM-AP のメモリ R/W だけで**ポーリングする方式。ターゲット側の協力は
制御ブロックを RAM に置くことだけで、割込みもデバッグ状態も不要
([SEGGER: About RTT](https://www.segger.com/products/debug-probes/j-link/technology/about-real-time-transfer/)、
[SEGGER KB: RTT](https://kb.segger.com/RTT))。
**コアが Running のままメモリを読めることは本プロジェクトで実証済み**
(MEM-AP はコア状態と独立。doc/verification-2026-07-13.ja.md の
Running 中 DHCSR ポーリングと同じ経路)なので、原理的な障害はない。

### 5.1 制御ブロックレイアウト

[SEGGER_RTT.c/h](https://github.com/SEGGERMicro/RTT/blob/main/RTT/SEGGER_RTT.c)
(probe-rs の実装
[rtt.rs](https://docs.rs/probe-rs-rtt/latest/src/probe_rs_rtt/rtt.rs.html)
も同じレイアウトを仮定)。32bit ターゲットでポインタ 4 バイト:

```text
SEGGER_RTT_CB(制御ブロック):
  +0   acID[16]            "SEGGER RTT" + NUL パディング(16 バイト)
  +16  MaxNumUpBuffers     (i32) 通常 3
  +20  MaxNumDownBuffers   (i32) 通常 3
  +24  aUp[MaxNumUpBuffers]     各 24 バイト
  +24 + 24*MaxNumUp  aDown[MaxNumDownBuffers]

SEGGER_RTT_BUFFER_UP / _DOWN(各 24 バイト):
  +0   sName        (const char*) 例 "Terminal"
  +4   pBuffer      (char*)  リングバッファ先頭
  +8   SizeOfBuffer (u32)
  +12  WrOff        (u32) 次に書く位置
  +16  RdOff        (u32) 次に読む位置
  +20  Flags        (u32) 下位 2bit: 0=SKIP, 1=TRIM, 2=BLOCK_IF_FIFO_FULL
```

- **Up バッファ(target→host)**: ターゲットが WrOff を進め、
  ホスト(=プローブ)がデータを読んで **RdOff を書き戻す**。
  有効データは `RdOff..WrOff`(mod Size)。
- **Down バッファ(host→target)**: プローブがデータを書いて
  **WrOff を進め**、ターゲットが RdOff を進める。
- どちらも索引 0 が "Terminal"(printf/scanf 相当)。defmt-rtt は
  up 1 本構成(MaxNumUp=1)。

プローブ側に必要な操作はワード/バイトのメモリ R/W のみで、
`ArmDebug::read_mem`/`write_mem` で完結する。

### 5.2 制御ブロックの発見

J-Link と同様に **SRAM を "SEGGER RTT" の 16 バイト ID で線形スキャン**する。
実装上の注意:

- 制御ブロックは実質ワードアライン(構造体)なので 4 バイト刻みで十分。
  probe-rs も範囲スキャン方式。
- 検索文字列そのものをプローブファームに平文で置くと、(将来 RTT を
  プローブ自身のログに使った場合等の)誤検出の種になるため、SEGGER の
  慣例に倣い分割/逆順で保持して実行時に組み立てる。
- スキャン範囲はターゲットの RAM 範囲(TargetFamily::memory_map)。
  RP2040: 0x2000_0000..0x2004_2000(264 KiB)。
  実測 RAM リード ~200 KB/s(verification-2026-07-13: Read 190〜204 KB/s)
  なので全域スキャンは **約 1.3 秒**。1 KiB ごとに読み(TAR wrap 単位)、
  チャンク境界跨ぎは 15 バイトのオーバーラップで処理。
- 高速化・確実化のため `monitor rtt setup <addr>`(GDB から
  `&_SEGGER_RTT` を直接指定。`monitor rtt setup 0x20000abc`)も用意する。
  ELF シンボルを知っている GDB 側から与えるのが最も確実。

## 6. ファームウェアへの統合方式の比較

### 案 (a): GDB コンソール(O パケット)への流し込み

`monitor rtt` 系コマンド+停止中/イベント時に O パケットで出力。

- RSP の O パケット(`$O<hex>#xx`)はスタブ→GDB の非同期出力だが、
  **all-stop の RSP では target running 中にスタブから任意パケットを
  送ることは想定されておらず**、gdbstub の state machine も Running 状態で
  ConsoleOutput を出す API を持たない(MonitorCmd の ConsoleOutput は
  コマンド応答中のみ)。
- → 「halt 中に溜まった分を `monitor rtt dump` で吸い出す」程度しか
  できず、"J-Link RTT 相当"(実行中の連続ログ)にならない。**不採用**
  (ただし M-RTT1 の足場・デバッグ手段としては有用)。

### 案 (b): RTT 専用の第 2 USB-CDC ポート(推奨)

プローブが USB 複合デバイスとして CDC-ACM を 2 本生やし、
2 本目を RTT 端末(`/dev/ttyACM1` を picocom 等で開くだけ)にする。

- **エンドポイント資源**: RP2040 の USB は EP0 以外に 15 エンドポイント×
  双方向分の DPRAM がある。CDC-ACM 1 本 = bulk IN/OUT + notify IN の 3 本。
  2 本でも合計 6 本+EP0 で余裕がある(rust-dap 本体は既に
  CDC×2+HID の複合構成の実績あり)。
- **usbd-serial**: 同一 `UsbBusAllocator` から `SerialPort::new` を 2 回
  呼ぶだけで複合化できる(ホスト互換性のため
  `UsbDeviceBuilder.composite_with_iads()` + IAD 対応ディスクリプタにする)。
- **ポーリング箇所**: `gdb_server.rs` の main ループは
  `GdbStubStateMachine::Running` アームで既に
  「USB pump → 1 バイト読む → `poll_stopped()`」を回している。ここに
  `rtt.poll(&mut target.arm, &mut rtt_serial)` を差し込む。
  Idle(halt 中)アームでもポーリングしてよい(halt 中も RAM は読める)。

### 案 (c): RTT-over-DAP(probe-rs 方式)

ホスト PC 側ツールが CMSIS-DAP 経由で RTT を読む方式。本構成は
**standalone GDB サーバでホストに probe-rs がいない**前提なので該当しない。
(rust-dap 本来の CMSIS-DAP ファームで probe-rs を使えば今日でも可能、
という注記だけ残す。)

**→ 推奨: 案 (b)**。GDB セッションと独立に RTT 端末が生き続け、
J-Link RTT Viewer 相当の使用感になる。

## 7. 案 (b) の具体設計

### 7.1 状態と構造

```rust
struct RttSession {
    /// 制御ブロック先頭(スキャン or monitor rtt setup で確定)
    cb: Option<u32>,
    /// up[0] / down[0] のバッファ記述子アドレス(cb+24, cb+24+24*max_up)
    up0: u32, down0: u32,
    /// キャッシュ: pBuffer/SizeOfBuffer(不変なので接続時に 1 回読む)
    up_buf: u32, up_size: u32, down_buf: u32, down_size: u32,
}
```

### 7.2 ポーリング(Running ループ内、1 イテレーションの仕事量を制限)

```text
rtt_poll():
 1. up0 の WrOff/RdOff を 1 回のブロックリード(8 バイト)で読む
 2. WrOff != RdOff なら連続領域分(リング折返しまで、かつ ≤64 バイト)を
    read_mem → CDC2 の TX キューへ → RdOff を書き戻す(write_word)
 3. CDC2 の RX に入力があり、down0 に空きがあれば ≤64 バイトを
    write_mem → WrOff を書き戻す
```

- 1 回のポーリングは最悪でも「8B 読み + 64B 読み + 4B 書き」程度に抑え、
  GDB の Ctrl-C 応答性・`poll_stopped()` の頻度を落とさない。
  ポーリングは数 100 µs〜1 ms 間隔(またはメインループ N 周に 1 回)。
- **コア選択との干渉**: RTT は共有 RAM への MEM-AP アクセスなので
  RP2040 ではどちらのコアの DP 経由でも読める(`cur_core` を触らない)。
  nRF52/54 は単一 DP なので問題自体がない。
- **SWD リンク断との干渉**: RTT ポーリング中のエラーは `diag[5]` に
  数えるだけで無視し、セッション再接続(`connect_and_halt`)時に
  制御ブロックを再スキャンする(ターゲットリセットで CB 位置は
  変わり得るため、`monitor reset` 後は再探索が必要)。

### 7.3 monitor コマンド

```text
monitor rtt              — スキャンして状態表示(CB アドレス、ch 数、名前)
monitor rtt setup <addr> — CB アドレス直接指定(スキャン省略)
monitor rtt stop         — ポーリング停止
```

### 7.4 性能見積り

実測値(doc/verification-2026-07-13.ja.md): ビットバング SWD で
RAM Read 190〜204 KB/s、Write 214〜231 KB/s、GDB `load`(フラッシュ)24 KB/s。

- ポーリング 1 回のオーバヘッド ≈ ヘッダ 8B 読み(数十 µs 台)+
  データ 64B 読み ≈ 0.4 ms 程度 → **1 kHz 弱でポーリングしても
  RTT スループット上限はおよそ 40〜60 KB/s**、実効はループ内の他仕事と
  折半して **~20 KB/s 程度**の見込み。ログ用途(数 KB/s)には十分。
  J-Link のような MB/s 級には届かない点は明記しておく。
- **PIO SWD トランスポート**(rp2040-pio ブランチ系)に載せ替えれば
  ワイヤレートが上がりそのまま効く。`ArmDebug` は `DapTransport`
  ジェネリックなので RTT 層に変更は不要。

### 7.5 リスクと対策

| リスク | 評価・対策 |
|---|---|
| Running 中の RAM 読み | 問題なし(MEM-AP はコア非依存、本プロジェクトで実証済み) |
| RdOff 書き戻しとターゲットの WrOff 更新の競合 | RTT の設計上、up バッファで host が書くのは RdOff のみ/target が書くのは WrOff のみ、と書き手が分離されており競合しない |
| GDB セッションポーリングとの干渉 | 同一ループ内でのインターリーブ。1 ポーリングの仕事量上限(≤64B)+ Ctrl-C バイトを常に先に処理 |
| ターゲット未初期化 RAM に偽 ID | ID 16 バイト完全一致 + MaxNumUp/Down の妥当性チェック(1..=16)+ pBuffer/Size が RAM 範囲内かの検証 |
| flash 書込中(flash_mode)の RTT | flash 操作中はポーリングを一時停止(`flash_mode` フラグ参照) |
| USB 複合化による Windows での COM ポート認識 | IAD 必須(composite_with_iads)。既存 rust-dap 複合構成の記述子を踏襲 |

### 7.6 マイルストーン(RTT)

| ID | 内容 | 検証 | 規模感 |
|---|---|---|---|
| **M-RTT1** | CB スキャン + `monitor rtt`(halt 中に up[0] を dump、O パケット出力)。RttSession 実装 | RP2040 ターゲットに rtt-target/defmt-rtt ファームを載せ、halt して dump | 小(1〜2 日) |
| **M-RTT2** | 第 2 CDC(IAD 複合化)+ Running ループでの up[0] ブリッジ | `blink + rtt` ファームで実行中ログが ttyACM1 に流れる。GDB 操作(break/step)との併用 | 中 |
| **M-RTT3** | down[0](ホスト→ターゲット入力)+ 再接続時の再スキャン + flash 中停止などの堅牢化 | 双方向端末(SEGGER の RTT terminal 相当)、`monitor reset` 跨ぎ | 小〜中 |
| **M-RTT4** | (任意)複数チャネル(defmt は ch0 のみで不要)、スループット計測・PIO 化の効果測定 | ベンチ | 小 |

RTT は nRF 対応と独立に RP2040 ターゲットだけで完結して作れる。
M-RTT2 完了時点で「J-Link RTT Viewer 相当(表示)」、M-RTT3 で入力込みの等価になる。

---

# 8. 全体推奨

1. **RTT(M-RTT1→2)を先行**: 依存が既存コードのみで、体感価値が大きい。
   arm-debug への変更ゼロ(read_mem/write_mem のみ使用)。
2. **nRF は M-nRF1(接続+APSEL)から**: nRF52840 DK 1 枚で
   コアデバッグ層の互換性を実証してから、TargetFamily リファクタ
   (M-nRF3)に進む。フラッシュは NVMC のおかげで RP2040 より簡単。
3. ARMv8-M(FPB rev2 / DWT v8-M / TrustZone CSW)は M-nRF5 に隔離、
   ADIv6(nRF54H)は将来課題として切り離す。

## 参考資料

- [nRF52840 Product Specification — NVMC](https://docs.nordicsemi.com/bundle/ps_nrf52840/page/nvmc.html)
- [nRF52840 Product Specification v1.5 (PDF)](https://files.seeedstudio.com/wiki/XIAO-BLE/nRF52840_PS_v1.5.pdf)
- [Nordic blog: Working with the nRF52 Series' improved APPROTECT](https://devzone.nordicsemi.com/nordic/nordic-blog/b/blog/posts/working-with-the-nrf52-series-improved-approtect)
- [Luca Bruno: nRF52 Debug Access Port protection](https://www.lucabruno.net/posts/2019-12-26/nrf52-ap-protection/)
- [nRF5340 Product Specification — CTRL-AP](https://infocenter.nordicsemi.com/topic/ps_nrf5340/ctrl-ap.html)
- [Nordic blog: Allowing debugger access to nRF5340](https://devzone.nordicsemi.com/nordic/nordic-blog/b/blog/posts/allowing-debugger-access-to-an-nrf5340)
- [nRF54L15 Datasheet — Debug and trace](https://docs.nordicsemi.com/bundle/ps_nrf54L15/page/debug.html) / [CTRL-AP](https://docs.nordicsemi.com/bundle/ps_nrf54L15/page/ctrl-ap.html) / [RRAMC](https://docs.nordicsemi.com/bundle/ps_nrf54L15/page/rramc.html)
- [OpenOCD patch: flash/nor: Add nRF54 NVM driver](https://www.mail-archive.com/openocd-devel@lists.sourceforge.net/msg17300.html)
- [nRF54H20 debugging — nRF Connect SDK docs](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/app_dev/device_guides/nrf54h/ug_nrf54h20_debugging.html)
- [ARMv7-M Architecture Reference Manual (DDI 0403)](https://developer.arm.com/documentation/ddi0403/d/)
- [Doulos: Using the Cortex-M3/M4 Flash Patch and Breakpoint unit (PDF)](https://www.doulos.com/media/1188/using_cortex-m3_fpb.pdf)
- [SEGGER: About Real Time Transfer](https://www.segger.com/products/debug-probes/j-link/technology/about-real-time-transfer/)
- [SEGGER KB: RTT](https://kb.segger.com/RTT)
- [SEGGER RTT reference source (SEGGER_RTT.c)](https://github.com/SEGGERMicro/RTT/blob/main/RTT/SEGGER_RTT.c)
- [probe-rs-rtt: rtt.rs source](https://docs.rs/probe-rs-rtt/latest/src/probe_rs_rtt/rtt.rs.html)
- 本リポジトリ: `doc/gdb-debugger-proposal.ja.md`, `doc/verification-2026-07-13.ja.md`(実測スループット)
