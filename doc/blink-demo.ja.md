# blink デモ — スタンドアロン GDB デバッガで LED を操る

プローブ Pico(gdb_server ファーム)経由で、ターゲット Pico に blink ファームを
書き込み、GDB から LED 点滅を観察・制御するデモ。M1〜M6 の機能
(load / ブレークポイント / ウォッチポイント / レジスタ・メモリ操作 / リセット)を
一通り体験できる。

## 構成

- プローブ: Raspberry Pi Pico + `gdb_server` ファーム(USB-CDC が `/dev/ttyACMx`)
- ターゲット: Raspberry Pi Pico(SWD 接続: プローブ GPIO2=SWCLK, GPIO3=SWDIO)
- ファーム: `boards/rpi_pico/src/bin/blink_demo.rs`
  - オンボード LED(GPIO25)を busy-wait ループで点滅
  - GDB から触れるグローバル変数(`#[no_mangle]` static):
    - `BLINK_ENABLED` — 0 で点滅停止、非 0 で再開
    - `BLINK_DELAY` — 半周期の busy ループ回数(≈6.5MHz ROSC 動作、500_000 ≈ 数 Hz)
    - `BLINK_COUNT` — トグル毎にインクリメント(ウォッチポイントの的)

## 0. ビルドと接続

```console
$ cd boards/rpi_pico && cargo build --release --bin blink_demo
$ gdb-multiarch target/thumbv6m-none-eabi/release/blink_demo \
    -ex 'set architecture armv4t' \
    -ex 'target remote /dev/ttyACM3'
```

ELF を渡して起動するとシンボル(`main`, `BLINK_*`)が使える。

## 1. 書き込み(GDB `load`)と実行

```gdb
(gdb) load                  # フラッシュへ書き込み(erase+program は透過)
(gdb) compare-sections      # 書けたことの照合(全 matched を確認)
(gdb) monitor reset         # リセット → LED が点滅を始める
```

## 2. 点滅を GDB から制御

走行中のターゲットに割り込むには Ctrl-C(バッチなら再 attach)。halt 中に:

```gdb
(gdb) set var blink_demo::BLINK_DELAY = 100000   # 5 倍速に
(gdb) continue                                   # → 速い点滅
^C
(gdb) set var blink_demo::BLINK_ENABLED = 0      # 点滅停止
(gdb) continue                                   # → LED 固定
```

注意: Rust モードの GDB ではグローバルは `blink_demo::BLINK_DELAY` のように
修飾パスで参照する(フレームによっては裸の名前が "No symbol" になる。
シングルクォートは Rust モードでは文字リテラル扱いなので使わない)。

## 3. LED を直接叩く(halt 中)

コアが halt していても SIO レジスタ書き込みで LED を直接操作できる:

```gdb
(gdb) set {unsigned int}0xd0000014 = (1 << 25)   # GPIO_OUT_SET → LED ON
(gdb) set {unsigned int}0xd0000018 = (1 << 25)   # GPIO_OUT_CLR → LED OFF
(gdb) x/1wx 0xd0000010                            # GPIO_OUT の現在値
```

(resume すると firmware がまた LED を上書きするので、観察するなら
`BLINK_ENABLED = 0` にしてから)

## 4. ブレークポイントとウォッチポイント

```gdb
(gdb) monitor reset halt          # リセットベクタ(bootrom 0xea)で halt
(gdb) hbreak main                 # フラッシュ上なので HW ブレーク
(gdb) continue                    # → main 先頭で停止(ソース行表示)
(gdb) watch blink_demo::BLINK_COUNT
(gdb) continue                    # → トグル 1 回ごとに停止(Old/New 表示)
(gdb) delete
```

## 5. リセット

```gdb
(gdb) monitor reset       # システムリセット → 再接続 → halt(位置は不定)
(gdb) monitor reset halt  # リセットベクタで halt(pc=0xea = bootrom 入口)
```

どちらもリセット後に SWD を張り直して halt 済みの状態で戻る。続けて
`continue` すれば bootrom → boot2 → ファームへとブートしていく。

## トラブルシューティング

- 変数が "No symbol": フレームの言語コンテキストの問題。`blink_demo::` 付きの
  修飾パスで参照する。
- ターゲットが暴走/不明な状態: `monitor reset halt` で確実に既知状態へ。
- スタブ内部の診断: `x/11wx 0xF0000000`(接続回数・エラーコード・リセット履歴)。

## 6. RTT ログ(J-Link RTT Viewer 相当)

blink_demo はトグル毎に RTT(ch0)へ 1 行出力する。プローブは CDC を 2 本持ち、
2 本目が RTT 端末になっている:

```gdb
(gdb) monitor rtt scan               # 制御ブロックを列挙(複数ある場合は選ぶ)
cb at 0x20000008: up=1 down=0 [valid]
(gdb) monitor rtt attach 0x20000008
(gdb) continue                        # ターゲットを走らせる
```

別端末で:

```console
$ picocom /dev/ttyACM4      # ← RSP の次の番号の CDC ポート
blink #123 led=1
blink #124 led=0
...
```

halt 中に溜まった分だけ見たい場合は `monitor rtt dump`。チャネル切替は
`monitor rtt channel <n>`(既定 0 = Terminal)。nRF Connect SDK のように
bootloader とアプリで別々の制御ブロックがある場合は scan が全部列挙するので、
生きている方(valid 表示・実行中に伸びる方)に attach する。

### 制御ブロックが見つからない(短命 RTT)

`monitor rtt scan` が `no control block found` を返すのに RTT を出しているはず、
という場合、制御ブロックは `_SEGGER_RTT` 初期化後に RAM 上へ現れるので、
**ブート直後の短時間だけ RTT が有効なファーム**(初期化後にブロックが上書き/
消去される等)だと scan の時点で通り過ぎていることがある。初期化済み・消去前の
窓で確実に捕まえる:

```gdb
(gdb) monitor reset halt          # リセットベクタで停止
(gdb) hbreak <RTT 初期化直後 / main 等>
(gdb) continue                    # 初期化を通った所で停止
(gdb) monitor rtt scan
(gdb) monitor rtt attach <addr>
(gdb) continue                    # 以降 第2 CDC でストリーミング
```

（nRF52 でこの手順で SEGGER RTT の実機動作を確認済み。RTT はプローブ側チップ
非依存なので RP2040/nRF52 どちらでも同じ操作。）
