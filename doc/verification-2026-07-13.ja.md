# fix/structural-issues ブランチ 実機検証記録 (2026-07-13)

対象コミット: 7024035 / 297e7a6 / b7ba938(Raspberry Pi Pico、デフォルト構成 = SWD + PIO)

## 構成

- プローブ: Raspberry Pi Pico(本ブランチのファームウェア、picotool で書き込み)
- ターゲット: ブランクの Raspberry Pi Pico(SWD 接続: GPIO2=SWCLK, GPIO3=SWDIO, GPIO4=RESET)
- UART ループバック: プローブの GPIO0(TxD) と GPIO1(RxD) をジャンパ直結
- ホストツール: probe-rs(CMSIS-DAP v2)、cdc_acm

## 結果

| # | テスト | 結果 |
|---|---|---|
| 1 | USB 列挙(CDC + Vendor 複合、MS OS 2.0 descriptor) | OK |
| 2 | probe-rs によるプローブ認識(DAP_Info) | OK |
| 3 | SWD 接続(DPv2 マルチドロップ、TARGETSEL) | OK |
| 4 | bootrom / SYSINFO CHIP_ID 読み出し(0x20002927 = RP2040 B2) | OK |
| 5 | RAM ベンチマーク(ベリファイ付き、32B〜32KB ブロック) | Read 180KB/s / Write 164KB/s |
| 6 | UART ループバック 32KB @115200bps | 全バイト一致 |
| 7 | UART ループバック 64KB @115200bps + DAP ベンチマーク同時実行 | 全バイト一致 |
| 8 | UART ループバック 256KB @921600bps + DAP ベンチマーク同時実行 | 全バイト一致・取りこぼしゼロ |
| 9 | CDC ライン設定変更による UART 再構成(115200→921600) | OK |

## 特記事項

- テスト8は UART 受信 約92KB/s に対し RX キューが 256 バイトのため、
  DAP 処理中に「キュー満杯 → RX 割り込み停止 → ドレイン後に再開」の
  フロー制御パス(297e7a6 で導入)を確実に通過している。データ損失なし。
- 旧構成(DAP 処理を USBCTRL_IRQ 内で実行)で懸念された
  「DAP 転送中の UART 取りこぼし」が解消されていることを確認した。
- UART ピンはデフォルト構成で GPIO0/GPIO1(UART0)。GPIO4/5 は uart1 feature
  (未マージの WIP)の割当てであり、SWD の RESET ピン(GPIO4)と衝突するため
  デフォルト構成では使用しない。

## 追記: v3 アーキテクチャの実機検証 (2026-07-13)

対象コミット: ccf2e8c(v3コア)/ 1f37d9e(rpi_pico bitbang+swd の v3 移行)。
`--features bitbang` ビルド(v3 `BitBangSwd` + `Dispatcher` + `v3::CmsisDap`)を
同一構成のプローブ/ターゲット/GPIO0-1 ジャンパで検証した。

| # | テスト | 結果 |
|---|---|---|
| 1 | USB 列挙・probe-rs 認識(v3 Dispatcher の DAP_Info 経路) | OK |
| 2 | SWD 接続と bootrom / CHIP_ID 読み出し(0x20002927) | OK |
| 3 | 4KB RAM 読み出し(v3 bitbang トランスポート) | OK(1024 ワード) |
| 4 | UART ループバック 8KB @115200(v3 CmsisDap と CDC の共存) | 全バイト一致 |

旧アーキテクチャ(PIO 構成)と v3 bitbang 構成の双方が同一ハードウェアで
動作することを確認した。残りの移行ステップ(PIO/JTAG の v3 移植、旧トレイト削除)は
doc/redesign-proposal.ja.md の §5 を参照。

## 追記2: v3 + PIO(デフォルト構成)の実機検証 (2026-07-13)

対象コミット: 2fe6c1e(PIO SwdIoSet/JtagIoSet の v3 アダプタ)。
デフォルト構成(SWD + PIO、v3 Dispatcher 経由)を同一ハード構成で検証した。

| # | テスト | 結果 |
|---|---|---|
| 1 | USB 列挙・probe-rs 認識 | OK |
| 2 | SWD 接続・CHIP_ID 読み出し(0x20002927) | OK |
| 3 | RAM ベンチマーク(ベリファイ付き) | Read 203KB/s / Write 231KB/s |
| 4 | UART ループバック 64KB @921600bps + DAP ベンチマーク同時実行 | 全バイト一致 |

ベンチマーク結果は旧スタック(Read 180KB/s / Write 164KB/s)より向上した。
v3 Dispatcher のコマンド処理経路が旧ブランケット実装チェーンより軽いことによる。
これで rpi_pico の jtag+bitbang を除く全構成が v3 で実機動作確認済みとなった。

## 追記3: レガシー削除後(0.3.0)の実機回帰検証 (2026-07-13)

対象コミット: bd2eb2d(レガシートレイトスタック削除、PIO の DapTransport ネイティブ化)。
デフォルト構成(SWD + PIO)のレガシーフリーファームウェアで回帰確認。

| # | テスト | 結果 |
|---|---|---|
| 1 | USB 列挙・probe-rs 認識 | OK |
| 2 | SWD 接続・CHIP_ID 読み出し(0x20002927) | OK |
| 3 | RAM ベンチマーク(ベリファイ付き) | Read 204KB/s / Write 225KB/s(アダプタ版と同等) |
| 4 | UART ループバック 64KB @921600bps + DAP ベンチマーク同時実行 | 全バイト一致 |

再設計提案のステップ1〜4が完了(v0.3.0)。残: §4 ボードレイヤ分離(boot2 移動、
UartBridge 共通化、workspace 化)、共有ピンでの SWD/JTAG 実行時切替(swj 構成)、
xiao_m0 の RTIC 化。

## 追記4: ステップ5(ボードレイヤ分離)の実機検証 (2026-07-13)

対象コミット: d6e8e84(boot2/pre_init のボード移管)/ 5d01aa6(bridge モジュール)/
22d0602(workspace 化)。

| # | テスト | 結果 |
|---|---|---|
| 1 | ブート成功(ボード提供の boot2 で起動) | OK |
| 2 | probe-rs 認識・CHIP_ID 読み出し | OK |
| 3 | RAM ベンチマーク | Read 190KB/s / Write 214KB/s |
| 4 | UART ループバック 64KB @921600bps(bridge モジュール経由) | 全バイト一致 |

これで再設計提案のステップ1〜5(§4 ボードレイヤ分離を含む)が完了。
残: 共有ピンでの SWD/JTAG 実行時切替(swj 構成)、xiao_m0 の BSP 更新+RTIC 化。

## 追記5: 共有ピン SWD/JTAG 実行時切替(swj)の実機検証 (2026-07-14)

対象コミット: b1273e8(BitBangSwj 結合トランスポート)。
rpi_pico を `--no-default-features --features swj` でビルドし、SWD ターゲット
(ブランク Pico)を接続して検証。

| # | テスト | 結果 |
|---|---|---|
| 1 | USB 列挙(`raspberry-pi-pico-swj`) | OK |
| 2 | capabilities = SWD|JTAG の広告 | OK(probe-rs が最初に JTAG を試行) |
| 3 | DAP_Connect→SWD 切替で CHIP_ID 読み出し(0x20002927) | OK |
| 4 | bootrom 読み出し | OK |

probe-rs が最初に JTAG プロービングを試みてから SWD にフォールバックした事実が、
1 つのファームが両プロトコルを広告できていること(=旧ブランケット実装では型
レベルで不可能だった構成)の証拠。JTAG スキャンチェーンのエラーは SWD ターゲット
に対する当然の結果で、SWD 切替後は正常。JTAG パス自体は JTAG ターゲット未接続の
ため実機未検証(ロジック/ビルド検証のみ)。

これで再設計提案の全項目(ステップ1〜5 + 共有ピン実行時切替)が実装・検証済み。

## 追記6: GDB デバッガ M1(arm-debug ADIv5)の実機検証 (2026-07-14)

対象コミット: 86ae941(arm-debug クレート)/ 0b464d2, b649ehclc 系(M1 self-test firmware)。
プローブ RP2040 に m1_selftest ファームを書き込み、bitbang SWD + arm-debug の
自前 ADIv5 スタックでターゲット RP2040 を読んだ。CDC 出力(DTR アサートで取得):

```
M1 dpidr=0x0bc12477 chipid=0x20002927 OK
```

| # | テスト | 結果 |
|---|---|---|
| 1 | USB-CDC 列挙(raspberry-pi-pico-m1) | OK |
| 2 | SWD 接続(line reset→dormant→SWD→マルチドロップ TARGETSEL→電源投入) | OK |
| 3 | DPIDR 読み出し = 0x0bc12477(正当な ARM Debug Port ID) | OK |
| 4 | MEM-AP 経由 CHIP_ID 読み出し = 0x20002927(probe-rs と一致) | OK |

arm-debug のホストテスト8件が実シリコンで裏付けられ、GDB デバッガ M1(ADIv5
bring-up)完了。TARGETSEL の ack 無しシーケンス、DP 電源投入ポーリング、MEM-AP
の CSW/TAR/DRW/RDBUFF 経路が実機で正しく動作。次は M2(Cortex-M コア制御:
halt/step/レジスタ)。

補足: Xous(Baochip 公式 OS)の USB は usb-device ベースで CDC-ACM を提供するが
役割排他(Views)であり、アプリからの任意 USB クラス追加は不可(HIDv2 = DAP v1
相当のみ抜け道)。カーネル gdbstub は USB-CDC ではなく物理 UART 専用。詳細は
doc/gdb-debugger-proposal.ja.md §8。

## 追記7: GDB デバッガ M2(Cortex-M コア制御)の実機検証 (2026-07-14)

対象コミット: 283077c(arm-debug M2)+ M2 self-test firmware。
m1_selftest ファームに halt/step/PC 読み出しを追加し、実ターゲットで検証:

```
M1 dpidr=0x0bc12477 chipid=0x20002927 OK | M2 halt OK pc0=0x00000030 pc1=0x00000032 stepped=true halted=true OK
```

| # | テスト | 結果 |
|---|---|---|
| 1 | コア halt(DHCSR C_HALT + S_HALT ポーリング) | OK |
| 2 | PC 読み出し(DCRSR/DCRDR、pc0=0x00000030) | OK |
| 3 | single-step(C_STEP + C_MASKINTS)で PC が 0x30→0x32 に前進 | OK(Thumb 1 命令 = 2 バイト) |
| 4 | step 後も halt 維持 / resume | OK |

arm-debug のホストテスト13件(M1: 8, M2: 5)が実シリコンで裏付けられ、GDB
デバッガ M2(Cortex-M コア制御: halt/step/レジスタ)完了。次は M3(gdbstub 統合、
USB-CDC 上で GDB `target remote` から RAM デバッグ)。

## 追記8: GDB デバッガ M3 切り分け — halt 状態でのレジスタ R/W (2026-07-14)

M3 で「GDB からのレジスタ/メモリ書き込みが失敗して見える」件の切り分け完了。
m1_selftest の M3 診断を修正し、コアレジスタアクセスを halt 状態で行うようにして
実機検証した(従来は m2_core_control の resume 後=走行中に実行しており、それが
失敗の原因だった):

```
M1 dpidr=0x0bc12477 chipid=0x20002927 OK | M2 halt OK pc0=0x00000030 pc1=0x00000032 stepped=true halted=true OK
M3 wword_ok=true readback=0xcafef00d halt_ok=true wreg_ok=true r0=0x11223344 WRITE_OK
```

| # | テスト | 結果 |
|---|---|---|
| 1 | MEM-AP write_word(0x20001000 ← 0xcafef00d)+ 読み戻し一致 | OK(コア走行中でも可) |
| 2 | 再 halt 後の write_core_reg(r0 ← 0x11223344) | OK |
| 3 | read_core_reg(r0)= 0x11223344 で書き込み値と一致 | OK |

結論: arm-debug の書き込み経路(MEM-AP / DCRSR・DCRDR)に欠陥なし。
コアレジスタアクセスは halt 必須(ADIv5/ARMv6-M 仕様どおり)、メモリアクセスは
MEM-AP 独立でコア状態に依らない。gdb_server は接続時・停止報告時に halt して
いるため、GDB からのレジスタ書き込みはこの前提を既に満たしている。

残: GDB(gdb-multiarch)からの通し確認(set $reg / メモリ書込 / break / continue)。
プローブ実機の USB が検証途中に物理的に応答不能となった(電源再投入・ハブ
リセットでも SETUP 無応答)ため、通し確認はハード復旧後に実施。

## 追記9: GDB デバッガ M3 通し確認 — RAM デバッグ MVP 達成 (2026-07-14)

gdb_server ファームに gdb-multiarch で接続し、RSP 経由の全デバッグ操作を実機で確認:

```
(gdb) set {unsigned short}0x20000000 = 0xbf00   # nop ×3 + b . を RAM に書く
0x20000000: 0xbf00 0xbf00 0xbf00 0xe7fe          # 読み戻し一致
(gdb) set $pc = 0x20000000 / set $sp = 0x20004000 / set $r0 = 0x55aa1234
(gdb) break *0x20000004
(gdb) continue
Breakpoint 1, 0x20000004 in ?? ()                # BKPT パッチで正しく停止
(gdb) print/x $r0      → 0x55aa1234              # 実行を跨いでレジスタ保持
(gdb) stepi            → pc 0x20000004 → 0x20000006
```

| # | テスト | 結果 |
|---|---|---|
| 1 | attach(halt 済みコア、pc/sp 読み出し) | OK |
| 2 | GDB からのメモリ書き込み(RAM プログラム 4 halfword)+ 読み戻し | OK |
| 3 | GDB からのレジスタ書き込み(pc/sp/r0) | OK |
| 4 | SW ブレークポイント(BKPT パッチ)+ continue で停止 | OK |
| 5 | ブレーク位置 pc=0x20000004、r0 値の保持 | OK |
| 6 | stepi(0x20000004→0x20000006) | OK |
| 7 | detach(即応答) | OK |

**GDB デバッガ M3 = RAM デバッグ MVP 完成**(read/write regs・read/write mem・
break・continue・step が GDB から通しで動作)。

既知の問題(切り分け済み・修正実装済み/実機未検証):

- **detach 後の再 attach でファームが停止**: detach 応答の残骸が TX キューに残った
  まま次セッションが始まり RSP が脱調 → gdbstub エラー → 旧実装は `loop_forever()`
  で USB ポーリングごと停止(ポートが完全無応答化)。対策: Disconnected 時に
  TX flush + RX drain(`purge`)、gdbstub エラー時は `SCB::sys_reset()` で自己復旧、
  step 完了の停止理由を DoneStep で報告。cable 復旧後に要再検証。
- **プローブ実機のコールドブート時 USB 列挙失敗**: VBUS 投入直後の SET_ADDRESS が
  EPROTO で失敗(warm リセット後は常に成功)。ポート電源再投入・ハブ再走査でも
  再現し、物理再接続で一時回復 → ケーブル/コネクタの信号品質を疑う。

## 追記10: GDB 再 attach の検証と SWD 接続シーケンス修正 (2026-07-15)

ケーブル再接続後、purge + sys_reset 版 gdb_server で検証:

| # | テスト | 結果 |
|---|---|---|
| 1 | フル E2E 再実行(追記9 と同一項目) | 全 OK |
| 2 | attach/detach ×5 連続 — ファーム停止(wedge) | 解消(5/5 で接続・切断成功) |
| 3 | attach/detach ×5 連続 — SWD リンク | **1 回おきに死亡**(奇数回で全アクセス NonFatal) |

3 の原因: `connect_multidrop` が line reset + dormant→SWD 選択しか送らず、
**既に SWD アクティブなターゲットに対して呼ぶと選択シーケンスで脱調**する。
失敗した試行が副作用でターゲットを dormant に落とすため、次の 1 回だけ成功する
(= 交互パターン)。修正: line reset 後に SWD→dormant 選択シーケンス(0xE3BC)を
送って既知状態から開始するようにした(ADIv5 の作法どおり)。ホストテスト 14 件
パス。実機検証はプローブ USB の物理不良(下記)により未了。

**ハード所見**: プローブの USB は「物理抜き差し直後は正常 → 数回の再列挙
(1200bps タッチ / ポート電源再投入)で SET_ADDRESS 無応答(EPROTO/-62)に陥り、
以後は物理抜き差しでしか回復しない」を 2 日連続で再現。ホスト側操作(ポート電源
断 3〜15 秒、PORT_RESET、ハブドライバ再バインド、ハブ USBDEVFS_RESET)はすべて
無効。ケーブル交換、または Pico の micro-USB コネクタのハンダ確認を要す。
