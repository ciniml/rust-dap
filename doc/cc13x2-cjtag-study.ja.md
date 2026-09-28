# CC13x2 (CC1312) を cJTAG 経由で GDB サーバから扱うための調査メモ

作成: 2026-09-29。対象: TI SimpleLink CC1312 (CC13x2 系、Cortex-M4F)。
出典: TI TRM SWCU185G 第 6 章、OpenOCD (`src/jtag/drivers/ftdi.c` の cJTAG 実装、
`tcl/target/ti/{icepick,cjtag,cc26x0}.cfg`、`src/flash/nor/cc26xx.c`、
`contrib/loaders/flash/cc26xx/`)、TI E2E。

## 1. 結論

- **GDB サーバ方式なら実現可能。** cJTAG はプローブ内で閉じ、GDB には RSP しか見えない。
  CMSIS-DAP と違いホスト側 (GDB / pyOCD / OpenOCD) の変更は不要。
- SWD は使えない (TRM: SWJ-DP だが SW-DP モードは未使用)。**JTAG-DP 一択**。
- 2-pin (TCKC/TMSC) の **OScan1** を実装する。4-pin 切替 (TDI/TDO = DIO16/17) も
  cJTAG コマンドで可能だが、基板に配線が無い前提で 2-pin を主経路とする。

## 2. デバッグサブシステムの事実 (TRM §6)

| 項目 | 内容 |
|---|---|
| 既定モード | POR 後は 2-pin cJTAG (IEEE 1149.7 Class 4)。TCKC/TMSC のみ |
| 4-pin | cJTAG コマンド窓を開き STC2 APFC=1 で切替。TDI/TDO は DIO16/17 に自動マップ |
| ルータ | ICEPick: IR 6 bit、IDCODE 0x0BB4102F (CC13x2/CC26x2)、POR 後はスキャンチェーンに ICEPick のみ |
| CPU DAP | Debug TAP 0、SWJ-DP (JTAG-DP のみ使用)、IDCODE 0x4BA00477、IR 4 bit |
| ロック | CCFG で CPU DAP を無効化可能 (無効なら接続不可) |
| ICEMelter | TCK 8 エッジで JTAG 電源ドメインを起床 |

チェーン順: 単一の secondary TAP 選択時「ICEPick の TDO に接続」= TDI → ICEPick → DAP → TDO。
実装では TLR 後の IDCODE 連続読み出しで順序を実測して確定する。

## 3. cJTAG (OScan1) の起動手順 (OpenOCD `cjtag_reset_online_activate` より)

TCKC=H のまま TMSC をトグルする「エスケープシーケンス」で状態を変える。

1. TMSC=0 で TCKC=1 → TMSC を **8 エッジ** (reset escape) → TCKC=0
2. TCKC を 3 パルス (パディング)
3. TCKC=1 で TMSC **6 エッジ** (select escape) → TCKC=0
4. TCKC の立ち上がりごとに TMSC に 12 bit を送る:
   OAC = 1100 (LSB first: 0,0,1,1)、EC = 1000 (0,0,0,1)、CP = 0000
   → OScan1 (2-wire) がアクティブ。JScan3 (4-wire) にする場合は OAC bit2 を 0 にする。
5. 以降、JTAG 1 bit = TCKC 3 サイクル:
   | サイクル | TMSC | 駆動側 |
   |---|---|---|
   | 1 | nTDI (反転 TDI) | プローブ |
   | 2 | TMS | プローブ |
   | 3 | TDO | ターゲット (プローブは Hi-Z) |

サンプリング: 既定 (STC1 SEDGE) ではターゲットは TCKC 立ち上がりで TMSC を取り込む。
プローブは TDO を TCKC 立ち上がり (3 サイクル目) で読む。実機で要確認。

## 4. cJTAG コマンド窓 (TRM §6.2.2、4-pin 切替や ECL に使用)

1. IR に BYPASS (0x3F) をロード (Pause-DR 終端)
2. ZBS (Capture-DR → Update-DR を Shift-DR を通らずに通過) × 2
3. 1 bit の Shift-DR → 制御レベル 2 で固定 (窓が開く)
4. コマンド = DR スキャン 2 回 (CP0 = Shift-DR のクロック数 = opcode、CP1 = オペランド)
   例: 4-pin 化 = STC2 (opcode 2) / APFC=1 → 2 bit → Update → 9 bit → Update
5. 閉じる: IR スキャン、TLR、または ECL (STMC opcode 0、CP1 = 1 bit)

## 5. ICEPick で CPU DAP を有効化 (OpenOCD `icepick_c_tapenable jrc 0`)

IR: BYPASS=0x00/0x3F、ROUTER=0x02、IDCODE=0x04、CONNECT=0x07。ROUTER DR (32 bit):
bit31 = W、bit30-28 = block (0 = ICEPick 制御、1 = Test TAP、2 = Debug TAP)、
bit27-24 = register、bit23-0 = value。

1. IR=CONNECT、DR 8 bit = 0x89 (接続キー)
2. ROUTER W block0 reg1 = 0x001000 (ICEPick 制御)
3. ROUTER W block2 reg0 = 0x110048 (ForceActive + InhibitSleep: 電源/クロック ON)
4. ROUTER W block2 reg0 = 0x112048 (デバッグ既定モード)
5. ROUTER W block2 reg0 = 0x112148 (SelectTAP=1)
6. IR=BYPASS で Run-Test/Idle に入る (ここで TAP が可視化) → runtest 10

注意: 手順中は Run-Test/Idle に入らない。TLR に入ると TAP 選択が解除される。

## 6. フラッシュ書き込み

- ROM API テーブル 0x10000180、`[10]` = フラッシュ API テーブル。
  `[5]` = FlashSectorErase(addr)、`[6]` = FlashProgram(src, addr, len)。
- セクタ 8 KiB。CC1312R1 は 352 KiB、R7 系は 704 KiB (`FLASH:FLASH_SIZE` レジスタ 0x4003002C で取得)。
- 実行前に VIMS (0x40034004) をキャッシュ OFF (|0x33) にし、STAT (0x40034000) bit3 の変化待ち。
  終了後に元へ戻す。ROM 関数は standby を無効化するので FLASH:CFG の DIS_STANDBY を戻す。
- OpenOCD は TI 製の常駐ローダ (BSD-3、ダブルバッファ) を RAM 0x20000000 に置く。
  本プロジェクトでは RP2040 の非同期ローダと同じ形で ROM API を呼ぶ小さな Thumb ローダを書く。

## 7. rust-dap 側の変更点

| 層 | 変更 |
|---|---|
| rust-dap `bitbang.rs` | OScan1 トランスポート: `JtagBits` の `write_bit/read_bit` (1 TCK) を 3 TCKC + TMSC 方向切替版に。IR/DR/DPACC/APACC 層は再利用 |
| rust-dap `bitbang.rs` | 既存 JTAG `transfer` が ACK (OK=010 / WAIT=001) を見ていないので判定を追加 |
| arm-debug | `transfer()` の JTAG-DP 経路 (DP 読みは RDBUFF の追加スキャンが必要)、`connect_jtag`、ICEPick ヘルパ |
| gdb-server-core | `Cc13x2Family`: connect (OScan1 起動 → ICEPick → DAP → power-up)、単一コア、ROM API フラッシュ |
| boards/rpi_pico | feature `gdb-target-cc13x2` + `cjtag` トランスポート選択。配線は TCKC=GPIO2、TMSC=GPIO3、nRESET=GPIO4 (SWD と同じ) |

## 8. リスク

- OScan1 のタイミング (TMSC の駆動/Hi-Z 切替、サンプリングエッジ) の実機合わせが本体。
- CCFG で DAP がロックされていると接続不可。
- Cortex-M4F の FPU レジスタは GDB に未公開 (必要なら後で追加)。
