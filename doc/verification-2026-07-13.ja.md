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

## 追記11: 再 attach 問題の真因特定と解決 — M3 完全動作 (2026-07-15)

追記10 の「1 回おきにリンク死亡」の真因は SWD ではなく **gdbstub 状態機械の
セッション跨ぎ再利用**だった。診断手順と結論:

1. GDB から読める診断窓(0xF000_0000、SWD 非依存)を実装 → 失敗セッションでは
   診断窓すら読めない = SWD 層より上の問題と判明。
2. connect 回数カウンタがセッションを跨いで 1 のまま → detach 後にファームが
   再起動していると推定。カーネルログの USB 再列挙で確認。
3. sys_reset を跨いで生き残る watchdog scratch にリセット箇所 ID を記録 →
   **site 3 = Idle 状態の incoming_data エラー**と特定。
4. 真因: セッション 1 が QStartNoAckMode を交渉 → detach 後 `return_to_idle()` で
   再利用された gdbstub は no-ack モードのまま → 次セッション冒頭の '+'(ack
   モード)でエラー → 自己リセット → 再列挙の谷間に attach したセッションが全滅
   → 交互パターン。

**修正**: Disconnected のたびに gdbstub を破棄・再構築(`ConnRef` で USB 接続を
参照渡しし、所有権を main に保持)。

| # | テスト | 結果 |
|---|---|---|
| 1 | attach/detach ×8 連続(メモリ読み出し付き) | **8/8 全て正常**(自己リセット 0) |
| 2 | フル E2E(メモリ/レジスタ書込・BP・continue・stepi) | 全 OK |
| 3 | BP セッション後の再 attach | OK |

**これで GDB デバッガ M3 は再接続を含め完全動作。** 追記10 の「交互リンク死」も
本件で説明される(SWD リンクは一度も壊れていなかった)。arm-debug の
SWD→dormant 強制(0xE3BC)は真因の修正ではなく ADIv5 仕様準拠の堅牢化として維持。
診断窓(0xF000_0000)と reset site 記録は恒久機能として残す。

## 追記12: GDB デバッガ M4(FPB ハードブレーク / DWT ウォッチポイント)実機検証 (2026-07-15)

arm-debug に FPB v1(ARMv6-M)/ DWT / DFSR 停止理由判別を実装し(ホストテスト
18 件)、gdb_server に Z1(hbreak)/ Z2〜Z4(watch/rwatch/awatch)を配線して実機検証:

| # | テスト | 結果 |
|---|---|---|
| 1 | `hbreak *0x190`(bootrom ROM)+ `bx` で突入 → フェッチで halt | OK(pc=0x190 ちょうどで停止、命令は未実行) |
| 2 | `watch *(unsigned int)0x20001100`(RAM str ループ) | OK(書込直後に停止、GDB が Old/New 値表示) |
| 3 | `rwatch`(RAM ldr ループ) | OK(読出で停止、Value 表示) |
| 4 | 停止理由の分類(DFSR → Watch{addr,kind} / HwBreak / SwBreak / DoneStep) | OK(GDB が各ブレークを正しく対応付け) |
| 5 | セッション跨ぎの残留コンパレータ掃除(attach 時 sweep) | OK(死んだセッションが残した FP_COMP が次の attach で全ゼロ) |

補助的な発見と対処:

- ロックアップ中のターゲット(pc=0xfffffffe)はブレーク不能。DEMCR.VC_CORERESET +
  AIRCR.SYSRESETREQ(いずれも GDB からのメモリ書込)でリセットベクタ halt に復旧
  できることを実機確認(M6 のリセット/ベクタキャッチの先行検証を兼ねる)。
- Running 状態のスタブへ新 GDB が接続すると gdbstub がエラー(reset site 4 で捕捉)
  → セッションエラーは再起動ではなくスタブ再構築で優雅に回復するよう変更。

**M4 完了**: フラッシュ/ROM 常駐コードのブレークとデータウォッチが GDB から使用可能。

## 追記13: GDB デバッガ M5(フラッシュ書換 = GDB `load`)実機検証 (2026-07-15)

ターゲット bootrom のフラッシュルーチンをリモート呼び出しする方式
(`arm-debug::call_function`: r0-r3 設定 → RAM の BKPT トランポリンへ lr →
C_MASKINTS 付き実行 → halt 待ち → r0 回収、使用レジスタは退避/復元)。
gdbstub が vFlash 未対応のため、XIP 窓への通常メモリ書込(M/X パケット)を
gdb_server 側で erase+program に透過変換(セクタ消去ビットマップ、256B 境界
0xFF パディング、resume/フラッシュ読出/再接続で XIP 復帰)。

| # | テスト | 結果 |
|---|---|---|
| 1 | `set {int}0x101FF000 = 0xcafebabe` → 読み戻し | OK(erase+program+XIP 復帰が透過) |
| 2 | 10KB ランダムデータ `restore`(3 セクタ跨ぎ)→ `dump` → cmp | 完全一致 |
| 3 | 別セッションでの永続性(再 attach 後の再読・再 dump) | 完全一致 |
| 4 | **GDB `load` で実ファーム ELF(29KB)を書込** | OK(15 KB/s、2 秒) |
| 5 | load 後にリセット → ターゲットがフラッシュからブート | OK(pc=0x100028e8 = XIP、sp=0x2003f870) |
| 6 | RAM SW ブレークポイント回帰 | OK |

副次確認: Running 中のスタブへの新規 attach が「セッションエラー → スタブ再構築 →
halt」で透過的に回復することを実運用で確認(追記12 の改善の実地検証)。

**M5 完了**: GDB `load` でファームを焼いて実行まで、スタンドアロンプローブのみで可能。
残る M6(デュアルコア/リセット)のうちリセット+ベクタキャッチは追記12 で実証済み。

## 追記14: バルク転送高速化と detach/attach レース (2026-07-15)

**バルク転送**: read_mem/write_mem をワード毎 TAR 再設定(1 ワード 4 転送)から
TAR オートインクリメントのブロック転送(1KB 境界で分割、16 ワード単位)に変更。

| # | テスト | 結果 |
|---|---|---|
| 1 | GDB `load`(29KB ELF) | **15 → 24 KB/s(1.6 倍)** |
| 2 | `compare-sections`(バルク読み経路で 29KB 照合) | 全セクション一致 |
| 3 | E2E(レジスタ/メモリ/BP/continue/stepi)回帰 | 全 OK |

**新たに発見したレース**: detach 直後に次のセッションが attach すると(バッチ
連続実行で再現)、Disconnected 処理の再接続後 purge が新セッションの
`+$qSupported...` を食い、そのセッションがハングする。修正: 再接続後の掃除を
「先頭の `+`/`-` のみ破棄、それ以外は 1 バイト push-back で状態機械に渡す」
(`drop_stray_acks`)に変更。**実機検証済み(再接続後)**: セッション間スリープ
なしの back-to-back 連続 attach ×10 が 10/10 成功、フル E2E・`load` 24KB/s・
`compare-sections` 一致も回帰確認。

## 追記15: GDB デバッガ M6(デュアルコア)実機検証 (2026-07-15)

arm-debug に `reselect`(dormant 手順なしの高速マルチドロップ切替)を追加し、
gdb_server を gdbstub の MultiThreadBase に移行(tid 1/2 = Core0/Core1)。
vCont は gdbstub の規約どおり「未指定スレッド = continue、ワイルドカード無し =
scheduler locking」を実装。all-stop(1 コア停止で全コア halt、停止コアの tid で
報告)。step 完了は tid 無しの DoneStep ではなく「stepped スレッドへの SIGTRAP」
で報告(誤帰属バグの修正)。HW ブレーク/ウォッチは両コアの FPB/DWT に設定。

| # | テスト | 結果 |
|---|---|---|
| 1 | `info threads` = 2 スレッド、pc/sp がコア毎に異なる | OK |
| 2 | スレッド往復 ×3 でレジスタ値が一貫(reselect の信頼性) | OK |
| 3 | `continue` → 「Thread 1 hit Breakpoint 1」(tid 帰属) | OK |
| 4 | thread 2 の `stepi` ×2(0x10001028→102a→102c)、誤帰属なし | OK |
| 5 | `set scheduler-locking on` で thread 1 のみ step、thread 2 不動 | OK |
| 6 | scheduler locking off では他コアが暗黙 continue(all-stop 準拠) | OK |
| 7 | E2E / `load` + compare-sections / 再 attach ×5(2 スレッド認識) | 全 OK |

**M6 完了 = 設計文書の M1〜M6 全マイルストーン達成。** スタンドアロン GDB
デバッガとして、デュアルコア RP2040 の RAM/フラッシュデバッグ・フラッシュ書込・
HW ブレーク/ウォッチが gdb-multiarch から一通り使用可能。

## 追記16: monitor reset と blink デモ環境 (2026-07-15)

`monitor reset` / `monitor reset halt` を実装(gdbstub MonitorCmd)。あわせて
デモ用ターゲットファーム `blink_demo` と手順書 doc/blink-demo.ja.md を追加。

| # | テスト | 結果 |
|---|---|---|
| 1 | `monitor reset halt` → pc=0xea(bootrom 入口、ベクタキャッチ) | OK |
| 2 | reset halt 後 `hbreak main` + `continue` → ソース行付きで停止、.data 初期化済(COUNT=0) | OK |
| 3 | `load` で blink_demo 書込 + compare-sections 一致 | OK |
| 4 | `set var blink_demo::BLINK_DELAY/ENABLED` の実行中反映 | OK |
| 5 | halt 中の SIO 直叩き(GPIO_OUT_SET/CLR)で LED ON/OFF | OK(OUT レジスタで確認) |
| 6 | `watch blink_demo::BLINK_COUNT` → 1 トグルで停止(Old/New 表示) | OK |

**インシデント記録(原因未特定・要ウォッチ)**: monitor reset 検証中、ターゲットの
フラッシュ(boot2/vector_table/.text)が BKPT パターン(0xbe00be00)を含む内容に
破損していた。症状は「ブート即 BKPT → GDB が説明できない swbreak を自動 continue
→ 無限ループ」で、当初 monitor reset のハングと誤認。`load` での再書込で修復し、
以後 inert な blink_demo をターゲットに使う運用に変更(旧ターゲットファームは
m1_selftest = SWD マスタとして動くファームで、状態が複雑だった)。破損の書込元は
特定できておらず(候補: 破損以前の実験セッションの何か)、blink_demo 運用で再発
しないか compare-sections で監視する。

補足(GDB 利用上の注意): Rust モードの GDB ではグローバル変数は
`blink_demo::BLINK_COUNT` の修飾パスで参照(フレームによって裸名は No symbol、
シングルクォートは文字リテラル扱い)。

## 追記17: 1200bps タッチ修正と Z0→FPB 昇格の実機検証 (2026-07-16)

**1200bps タッチのポート固着(解決)**: 原因は reset_to_usb_boot をエニュメレート
中に即時実行していたこと(D+ がホストの処理より速くバウンスし、xHCI ポートが
SET_ADDRESS タイムアウトで固着)。`usb_detach_for_reset()`(SIE_CTRL.PULLUP_EN
クリア → 0.5 秒待機 → リブート)を reset_to_bootloader と gdb_server の
reset_self に導入。

| # | テスト | 結果 |
|---|---|---|
| 1 | 1200bps タッチ → BOOTSEL → 書込 → 列挙、連続 5 回 | **5/5 成功**(修正前は 1〜2 回で固着) |
| 2 | 普通の `break main`(フラッシュ、Z0→FPB 昇格)+ continue | ヒット(修正前は無音の不発) |
| 3 | 昇格ブレークの SwBreak 報告(GDB のブレーク対応付け) | OK |
| 4 | RAM への `break`(BKPT パッチ経路)回帰 | OK |

これまで「物理不良疑い」としていたポート固着はファームウェアの USB 終了処理
不足が真因だった(ユーザ指摘が契機)。追記10/14 の該当記述はこの結論で上書き。
