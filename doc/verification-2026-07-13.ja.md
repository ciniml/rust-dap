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
