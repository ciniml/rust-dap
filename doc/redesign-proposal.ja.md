# rust-dap 再設計提案

対象: トレイト階層の再設計(構造調査レポート項目3)およびボード抽象化レイヤの分離(項目4)。
ステータス: ステップ1〜5 実装済み(v0.3.0、実機検証済み。doc/verification-2026-07-13.ja.md 参照。移行期の v3 モジュールはクレートルートへ統合済み)。残: 共有ピン SWD/JTAG 実行時切替(swj 構成)、xiao_m0 の BSP 更新+RTIC 化。

## 1. 現状の問題(要約)

1. `BitBangSwdIo → PrimitiveSwdIo → SwdIo → CmsisDapCommandInner → CmsisDapCommand` と
   全段がブランケット実装で連鎖しており、コヒーレンス規則により
   **1つの型が SWD 用と JTAG 用の実装を同時に持てない**。
   その結果 SWD/JTAG が cargo feature で排他になっており、
   CMSIS-DAP 本来の DAP_Connect(port) による実行時切替ができない。
2. コマンドのバイト列パースがコア側(`jtag_sequence`)と実装側(`swd_sequence`)に
   分散しており、実装を追加するたびにパースが重複する。
3. embedded-hal 0.2 の `IoPin<I, O>`(1.0 で廃止)にピン抽象が依存しており、
   ピン1本につき入力型+出力型の2型引数が必要
   (`SwdIoSet` で7個、`JtagIoSet` で13個)。embedded-hal 1.0 へ移行できない。
4. リトライポリシー・エラー変換などのプロトコル方針が各実装に重複している。

## 2. 設計目標

- DAP_Connect の port 引数で SWD/JTAG を実行時に切替可能にする
- ブランケット実装の連鎖を廃止し、部分的なオーバーライド(例: transfer だけ PIO 化)を可能にする
- コマンドパースを100%コア側に集約する。物理層はパース済みの構造体だけを受け取る
- embedded-hal 1.0 ベースでピン抽象を「1ピン1型」にする
- パニック/UB を排除し、すべてエラー戻り値にする(fix/structural-issues で一部済み)
- no_std / ゼロアロケーション維持

## 3. 新アーキテクチャ

```
CmsisDap<'a, B, T, const N>        … USBクラス。パケット分割/結合のみ(現行踏襲)
  └─ Dispatcher                    … コアの具象コード(trait ではない)
        ├─ コマンドパース(全コマンド)
        ├─ リトライ/マッチマスク等のプロトコルポリシー
        └─ trait DapTransport      … 物理層抽象(唯一の実装ポイント)
              ├─ SWD 系メソッド(デフォルト実装 = NotSupported)
              └─ JTAG 系メソッド(デフォルト実装 = NotSupported)
```

### 3.1 DapTransport トレイト

「SWDのみ」「JTAGのみ」「両対応」を1つの型で表現できるよう、
細粒度メソッド+デフォルト `Err(NotSupported)` 方式を採る。
ブランケット実装は用いない。

```rust
pub enum ConnectPort { Default, Swd, Jtag }
pub enum ActivePort { Swd, Jtag }

pub trait DapTransport {
    /// このトランスポートが実際にサポートする機能。DAP_Info Capabilities に反映される。
    fn capabilities(&self) -> DapCapabilities;

    /// DAP_Connect。port に応じてピンを再構成し、実際に選択した port を返す。
    fn connect(&mut self, port: ConnectPort, config: &DapConfig)
        -> Result<ActivePort, DapError>;
    fn disconnect(&mut self, config: &DapConfig) -> Result<(), DapError>;

    // ---- SWJ 共通(必須) ----
    fn swj_sequence(&mut self, config: &DapConfig, count: usize, data: &[u8])
        -> Result<(), DapError>;
    fn swj_pins(&mut self, config: &DapConfig, output: SwjPins, select: SwjPins, wait_us: u32)
        -> Result<SwjPins, DapError>;
    fn swj_clock(&mut self, config: &mut DapConfig, frequency_hz: u32)
        -> Result<(), DapError>;

    // ---- SWD(SWD対応トランスポートのみ実装) ----
    fn swd_transfer(&mut self, config: &DapConfig, req: SwdRequest, data: u32)
        -> Result<u32, DapError> { Err(DapError::NotSupported) }
    fn swd_read_bits(&mut self, config: &DapConfig, count: usize, out: &mut [u8])
        -> Result<(), DapError> { Err(DapError::NotSupported) }
    fn swd_write_bits(&mut self, config: &DapConfig, count: usize, data: &[u8])
        -> Result<(), DapError> { Err(DapError::NotSupported) }
    fn swd_output_enable(&mut self, enable: bool) -> Result<(), DapError> { Ok(()) }

    // ---- JTAG(JTAG対応トランスポートのみ実装) ----
    fn jtag_transfer(&mut self, config: &DapConfig, dap_index: u8, req: SwdRequest, data: u32)
        -> Result<u32, DapError> { Err(DapError::NotSupported) }
    fn jtag_sequence(&mut self, config: &DapConfig, info: &JtagSequenceInfo, tdi: u64)
        -> Result<Option<u64>, DapError> { Err(DapError::NotSupported) }
    fn jtag_idcode(&mut self, config: &DapConfig, index: u8)
        -> Result<u32, DapError> { Err(DapError::NotSupported) }
}
```

ポイント:

- **実装は各ボード/方式につき1型だけ**(例: `Rp2040PioSwj`、`BitBangSwj<PINS>`)。
  SWD/JTAG でピンを共有するハードウェア(GPIO2/3 が SWCLK/TCK, SWDIO/TMS を兼ねる等)は
  ピン一式をこの1型が所有し、`connect(port)` の中で再構成する。
  現行の「SwdIoSet と JtagIoSet が同じピンを取り合うので型レベルで排他」問題が消える。
- SWD Sequence / JTAG Sequence 等のリクエストパースは Dispatcher が行い、
  トランスポートには `read_bits`/`write_bits`/`jtag_sequence(info, tdi)` のような
  パース済みの単位だけが渡る(現行 `swd_sequence` の request/response 生渡しを廃止)。
- `transfer_inner_with_retry`・マッチマスク・posted read の管理は Dispatcher に一本化。
  トランスポートの `swd_transfer`/`jtag_transfer` は1回分の転送だけを行う。
- `DapError` に `NotSupported` を追加(応答は DAP_ERROR)。`SwdError(u8)` は
  `TransferAck(u8)` に改名し SWD/JTAG 共用であることを明示。

### 3.2 Dispatcher(コア具象コード)

```rust
pub struct Dispatcher {
    config: DapConfig,
    active_port: Option<ActivePort>,
}

impl Dispatcher {
    pub fn execute<T: DapTransport>(
        &mut self,
        transport: &mut T,
        command: DapCommandId,
        request: &[u8],
        response: &mut [u8],
    ) -> Result<(usize, usize), DapError> { /* 現行 CmsisDapCommand の中身を移植 */ }
}
```

- 現行の `CmsisDapCommand`/`CmsisDapCommandInner` は削除。トレイトを介した
  カスタマイズポイントは `DapTransport` の1箇所になる。
- ベンダ拡張コマンド(0x80-0x9F)用に、必要になった時点で
  `trait VendorCommands`(デフォルト no-op)を `CmsisDap` に追加できる余地を残す。

### 3.3 DapConfig と識別情報

```rust
pub struct DapIdentity {
    pub vendor: &'static str,
    pub product: &'static str,
    pub serial_number: &'static str,
    pub firmware_version: &'static str,
    pub packet_count: u8,
}

pub struct DapConfig {
    pub identity: DapIdentity,
    pub swd: SwdConfig,      // 旧 SwdIoConfig
    pub jtag: JtagConfig,    // 旧 JtagIoConfig
    pub retry_count: u32,
    pub match_mask: u32,
    pub match_retry_count: u32,
    /// swj_pins の wait_us 換算等に使う。ボードの実クロックを渡す。
    pub core_clock_hz: u32,
}
```

- `CmsisDap::new(alloc, transport, config)` で注入。"Hoge/Fuga/Piyo"、
  `CORE_CLOCK` 決め打ち(bitbang.rs)、`CPU_FREQUENCY_HZ` 決め打ち(util.rs)を全廃する。
- `JtagConfig::ir_length: [u8; 256]`(256バイト)は `[u8; MAX_JTAG_DEVICES]`
  (デフォルト8、const generic 化は不要)に縮小する。
  現実のスキャンチェーンで256デバイスは想定不要で、RAM 節約になる。

### 3.4 ピン抽象(embedded-hal 1.0 対応)

`IoPin` の代替として双方向ピンを1型で表すトレイトを rust-dap 側に定義する:

```rust
pub trait BidirPin {
    type Error;
    /// 出力モードへ切替え、同時に初期値を駆動する
    fn into_output(&mut self, state: PinState) -> Result<(), Self::Error>;
    /// 入力(Hi-Z)モードへ切替える
    fn into_input(&mut self) -> Result<(), Self::Error>;
    fn write(&mut self, state: PinState) -> Result<(), Self::Error>;
    fn read(&mut self) -> Result<bool, Self::Error>;
}
```

- モード切替を「型変換」ではなく「メソッド呼び出し」にすることで、
  Input/Output の `Option` ペア持ち回りと panic を排除する。
- RP2040 では `Pin<DynPinId, FunctionSio<SioOutput>, PullNone>` 系
  (rp2040-hal 0.9+ の DynPin)で実装。SAMD21 も同様に DynPin で実装可能。
- これにより型引数は `BitBangSwj<CLK, DIO, RST, DELAY>`(4個)、
  JTAG 込みでも `BitBangSwj<TCK, TMS, TDI, TDO, TRST, SRST, DELAY>`(7個)に減る。
  さらに全ピンを同一 DynPin 型にすれば `BitBangSwj<P: BidirPin, DELAY>` +
  `[P; N]` まで簡約できる(ボード側の記述量最小。性能面は §6 参照)。

### 3.5 遅延抽象

`DelayFunc` は概ね現行を維持しつつ、周波数→サイクル換算を config 経由にする:

```rust
pub trait CycleDelay {
    fn delay_cycles(&self, cycles: u32);
    /// core_clock_hz を config から受け取れるため、実装は定数を持たない
    fn half_clock_cycles(core_clock_hz: u32, target_hz: u32) -> u32 { ... }
}
```

## 4. ボード抽象化レイヤの分離(項目4)

### 4.1 rust-dap-rp2040 の責務整理

| 現状 | 提案 |
|---|---|
| `BOOT2_FIRMWARE` をライブラリが無条件提供 | ボードクレートへ移動。移行期は `boot2-w25q080` / `boot2-ram-memcpy` feature(default off)として残す |
| `#[pre_init]`(スピンロック初期化)を無条件提供 | `pub unsafe fn clear_spinlocks()` として公開し、ボード側の `#[pre_init]` から呼ぶ |
| `initialize_usb` が VID/PID・文字列を決め打ち | `UsbIdentity` 引数化(fix/structural-issues で追加済みの `initialize_usb_with_identity` を正式APIに) |
| RTIC app が各ボードに丸ごとコピペ | 下記 `bridge` モジュールへ吸い上げ |

### 4.2 USB-UART ブリッジの共通化

RTIC の `#[app]` はマクロの制約でライブラリ化できないが、タスク本体は関数化できる。
rust-dap-rp2040 に `bridge` モジュールを新設する:

```rust
pub struct UartBridge<const RX_N: usize, const TX_N: usize> {
    // heapless queue の producer/consumer と UartReader/Writer を所有
}

impl UartBridge {
    pub fn on_uart_irq(&mut self, ...);        // 現 uart_irq 本体(RXフロー制御込み)
    pub fn pump(&mut self, usb_serial: &mut SerialPort<UsbBus>);  // 現 idle/usb 側の3ループ
    pub fn apply_line_coding(&mut self, ...);  // 現 UART 再設定ブロック
}
```

ボードの main.rs は「ピン/UART/PIO の型定義」「init での結線」「RTIC タスクから
bridge メソッドを呼ぶ数行」だけになる(現状約480行 → 目標150行程度)。
rpi_pico と xiao_rp2040 の差分はピン割当てと `UsbIdentity` のみになる。

### 4.3 workspace 化

ルートに virtual workspace を置き、ホストでビルド可能なクレートだけを
`default-members` にする:

```toml
[workspace]
members = ["rust-dap", "rust-dap-rp2040", "boards/rpi_pico", "boards/xiao_rp2040", "boards/xiao_m0"]
default-members = ["rust-dap"]
resolver = "2"
```

- `cargo test` / `cargo clippy` がルートで完結(CI 簡素化)。
- ボードは従来通り各ディレクトリの `.cargo/config.toml` がターゲットを指定するので、
  `cd boards/rpi_pico && cargo build` の使い勝手は変わらない。
- Cargo.lock が1本化され、cortex-m 0.6/0.7 混在のようなバージョンずれが可視化される。
- リポジトリ直下に散乱している worktree ディレクトリ群は `.gitignore` へ追加する。

### 4.4 xiao_m0 の扱い

- `static mut` + 割り込みの現行構成は Rust 2024 で維持不能。atsamd-hal の
  RTIC 対応版へ書き換えるか、メンテ対象から外すかの判断が必要(要オーナー判断)。
- 書き換える場合は §4.2 の bridge を SAMD 用に薄く実装し直す
  (UART ブリッジ機能自体が現状無いので、DAP のみなら小規模)。

## 5. 移行計画(各ステップでビルド/実機確認可能)

1. **コアに新API追加**(旧APIと共存): `DapTransport` + `Dispatcher` + `DapConfig` を
   `rust_dap::v3` モジュールとして追加。既存テスト維持+Dispatcher の
   ホストテストを拡充(パース系は全コマンドをホストでテスト可能になる)。
2. **bitbang SWD を移植**して rpi_pico(bitbang, swd)を新APIに切替え、実機確認。
3. **PIO SWD/JTAG を移植**。`connect(port)` で PIO プログラムを install し直す
   実装にし、SWD/JTAG 実行時切替を実機確認(openocd / probe-rs / pyOCD)。
4. **旧トレイト削除**、embedded-hal 1.0 / rp2040-hal 0.10+ へ依存更新。
   rust-dap 0.3.0 としてタグ付け。
5. **ボードレイヤ整理**(§4): boot2 移動 → bridge 抽出 → workspace 化。
   RTIC 2.x への移行はこの後の独立タスクとする(必須ではない)。

ステップ1-2 が最小の検証可能単位。3以降は実機がないと進められない。

## 6. 検討した代替案

- **enum ディスパッチ(`DapPort<S, J>`)**: SWD/JTAG を別型にして enum で切替える案。
  ピンを共有するボードで両ドライバがピン所有権を取り合うため不成立。却下。
- **`&dyn DapTransport`(トレイトオブジェクト)**: 型引数は消えるが、
  bitbang の `write_bit` はホットループでの動的ディスパッチとなり
  SWCLK 速度が数割落ちる懸念。静的ディスパッチを維持する。
- **全ピン DynPin 化(§3.4 末尾)**: 記述量は最小だが、SIO レジスタ直叩きに比べ
  ピン番号のランタイム分岐が入る。bitbang はどのみち遅い(高速化は PIO の役目)ため
  **採用寄り**だが、ステップ2の実機計測で判断する。

## 7. オープンな論点(要オーナー判断)

1. VID/PID: 0x6666/0x4444(プロトタイプID)のままで良いか。pid.codes 取得の意向は?
2. xiao_m0 を RTIC 化して維持するか、アーカイブするか(§4.4)。
3. 旧API(0.2系トレイト)の互換をどこまで残すか。crates.io 利用者
   (elfmimi/tnishinaga 派生など)向けに 0.2 ブランチを残すか。
4. `set_clock` feature は新設計では常時有効(swj_clock 実装必須)にして
   feature 自体を廃止したいが問題ないか。
