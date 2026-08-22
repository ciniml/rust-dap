# LT: Bao Chipでデバッガつくってみた

- 持ち時間: 5分
- 題材: rust-dap / Baochip-1x / Dabao board
- 想定スライド枚数: 8〜10枚(1枚あたり約30秒)

---

## 0. タイトル・自己紹介(15秒 / 1枚)

- タイトル案: 「Rust製デバッグプローブ rust-dap を Baochip に移植する」
- 名前・アイコン・一言(組込みRust / デバッグプローブ自作の人、など)

---

## 1. Baochip / Dabao board とは(60秒 / 1〜2枚)

### スライド: Dabao board の紹介

- **Baochip-1x** SoC と、その評価ボード **Dabao**($12)の紹介
- 作者は **bunnie**(Andrew "bunnie" Huang): 著書『**ハードウェアハッカー** 〜新しいモノを
  つくる破壊と創造の冒険』(技術評論社、2018)で知られるハードウェアハッカー
- セキュリティ設計で特徴的な部分(TRNG・セキュアブートは昨今当たり前なので挙げない):
  - **RTL がほぼオープン**(Mostly open RTL)
  - **裏面から赤外線で回路を透視できる**: シリコンは近赤外に透明なので、実装したまま
    裏面から撮像して実チップが公開 RTL 通りか非破壊で検証できる(バックドア・改竄の検出)。
    比較表では他チップはすべて No
  - グリッチセンサ、ECC 保護 RAM、セキュアメッシュ、一方向カウンタ
- 「OSS で検査可能」の意味 = **設計(RTL)が読める** × **実物が設計通りか自分で確かめられる**。
  ソース公開だけでなく、製造されたシリコンまで検証の対象にしているのが特徴
- **ボードの写真を必ず入れる**(現物感が伝わるので)

### スライド: Baochip-1x の特徴的な構成

- CPUコア: **VexRiscv**(RV32IMAC + MMU、350MHz、SpinalHDL 製のソフトコア系列)
- **MMU 有効** — マイコンクラスなのに MPU でなく MMU を持つ
  - 狙いはセキュリティ: 全プロセスが独立した仮想メモリ空間で動く。
    「Rust の所有権・借用のルールを、プロセス間でも MMU で強制する」設計思想
    (Xous の IPC: Borrow=`&T` / MutableBorrow=`&mut T` / Send=move を
    ページテーブルの付け替えで実現。move したページは送り手からアンマップされる)
  - その上で **Xous** が動く
- **Xous** とは:
  - Pure Rust のマイクロカーネル OS(元は秘匿通信端末 Precursor 用に開発)
  - すべてがプロセス分離+メッセージパッシング。ドライバもユーザ空間の
    「サービス」で、アプリは IPC 経由でペリフェラルを触る
  - つまり **GPIO を1回叩くたびに IPC** ← **後半(§4 の遅さの正体)への伏線**
- **BIO** — RP2040 の PIO 相当のプログラマブル I/O エンジン
  - **PicoRV コア ×4 @ 700MHz** で構成(PIO と違い普通の RISC-V 命令で書ける)
  - CAN / LED ストリップ / USB FS / I2C / UART / SPI 等を実装可能、オープンソース
- GPIO の最大動作速度: **BIO トグルレートで最大 25MHz**
- メモリ: 4MiB RRAM(オンチップ ROM)+ 2MiB SRAM

(出典: 開発者自身の紹介ページ <https://baochip.com/dabao-intro/>)

> 話の組み立て: 「MMU+マイクロカーネルで硬い」×「BIO で速い I/O」という
> 2本柱をここで立てておき、後半(遅い→BIOで解決)で回収する。

---

## 2. rust-dap とは(45秒 / 1枚)

- Rust 製の CMSIS-DAP デバッグプローブファームウェア
- 対応ボード: Raspberry Pi Pico、XIAO RP2040、XIAO M0 など
- 最近の発展: プローブ単体で GDB RSP を喋る**スタンドアロン GDB サーバ**機能
  (Black Magic Probe 相当。gdbstub + arm-debug + gdb-server-core)

### アーキテクチャ図(1枚)

```
GDB (host)
  │ RSP over USB CDC-ACM
  ▼
[プローブ]
  Connection 実装        ← ボード固有
  gdbstub state machine  ← セッションループ(ボード固有)
  GdbTarget              ← gdb-server-core(ボード非依存)
  ArmDebug (ADIv5/CM)    ← arm-debug(ボード非依存)
  DapTransport           ← ボード固有
  │ SWD
  ▼
[ターゲット MCU]
```

---

## 3. Baochip への移植(90秒 / 2枚)

### スライド: 移植で書いたのは3つだけ

- コア(`gdb-server-core` / `arm-debug`)はボード非依存に切り出し済み
- 書いたのは **Connection・DapTransport・セッションループの3つだけ**
- 前ページのアーキテクチャ図を再掲し、ボード固有部分だけ色を付けると分かりやすい

### スライド: Xous アプリならではのハマりどころ(1〜2個に厳選)

- USB シリアルはすでに Xous のサービス(別プロセス)として動いているので、アプリからの
  送受信はそのサービスとの IPC になる(ここが Xous 固有。送受信を別スレッドで待つこと
  自体は珍しくない)。読みはブロックするが gdbstub は非ブロック読みを要求
  → 受信専用スレッドが別コネクションで IPC 待ちし、キューに積む構成に
- そもそも GPIO をメイン CPU が直接叩く前提ではない
  - GPIO は hal-service が所有し、アプリからは 1 操作 = 1 IPC。細かい I/O は BIO に
    任せる設計思想
  → まずは IPC 越しのビットバング SWD で動かす(高インピーダンス初期化・方向より先に
  レベル設定、などの工夫)

### スライド: 動いた画面

- GDB から接続しての実行結果(IPC 越しビットバング SWD、2026-08-22 計測)を表で掲載:
  | コマンド | 内容 | 時間 |
  |---|---|---|
  | `x/1wx 0x10000000` | 1 ワード読み | 9.1 ms |
  | `dump binary memory r1k.bin 0x10000000 0x10000400` | 1 KiB 読み | 815 ms (1.23 KiB/s) |
  | `dump binary memory r16k.bin 0x10000000 0x10004000` | 16 KiB 読み | 13026 ms (1.23 KiB/s) |
  | `set {int}0x20000000 = 0xdeadbeef` | 1 ワード書き | 618 ms |
  | `restore r4k.bin binary 0x20000000` | 4 KiB RAM 書き | 3274 ms (1.22 KiB/s) |
  | `stepi` | ステップ実行 | 1.1〜1.3 s |
- ✅ 2026-08-22 に実機 E2E 全 PASS(handshake、halt、2スレッド列挙、レジスタ/メモリ読み、
  RAM 書き込み、stepi、hbreak、continue、monitor)。スクショ素材を撮ればよい状態

---

## 4. BIO によるパフォーマンス改善(60秒 / 1〜2枚)

### スライド: 課題 — 遅い

- SWD のエッジごとに hal-service への IPC が入る
- 実測(2026-08-22 ベースライン): メモリ読み出し **1.23 KiB/s**(サイズ非依存、ワード毎 ~3.2ms の
  IPC コストが支配的 ≒ SWCLK 実効 ~30kHz)。RAM 書き込みも同速 1.22 KiB/s
- 「MMU+マイクロカーネルの代償」として §1 の伏線をここで回収

### スライド: 解決 — BIO オフロード

- SWD シーケンスを BIO(RP2040 PIO 相当)にオフロード
- rust-dap には RP2040 PIO 版 SwdIoSet の前例あり(`rp2040-pio` ブランチ)、設計をなぞれる
- hal-service に BIO の claim/grant API が整備済み(`claim_dynamic_pin` 等)
- **発表時点の BIO 版の状況**: BIO オフロード版 SWD を実装し、BIO 側の動作(出力)まで確認済み。
  ターゲットからの信号入力がまだ読めておらず、設定は一次資料と照合済み → 物理観測
  (ロジアナ/オシロ)で決着予定。スライドは「実測 1.23 KiB/s → BIO オフロード(状況)」の構成
- SWD における BIO の利点:
  - RV32E なので PIO より複雑な処理(ACK 判定、パリティ計算)が可能
  - ハードウェア制御機構による細かいタイミング制御
- 参考: rust-dap には RP2040 PIO 版 SWD の前例あり(設計をなぞれる)
- BIO 版が完成したら After 側を実測値に差し替える

---

## 5. まとめ(30秒 / 1枚)

- Baochip = **MMU 付き RISC-V + BIO** という面白い構成
- rust-dap は層分離のおかげで**移植コストが小さい**(書くのは3ファイル相当)
- **BIO はいいぞ!** — RV32E で書ける PIO。SWD に限らず「ちょっと賢い I/O」全般に効く
- Dabao の購入: Crowd Supply <https://www.crowdsupply.com/baochip/dabao>
  $12/枚(送料 US $10・海外 $18)、予約受付中、現在の予約分は 2026-10-09 出荷予定
  (2026-08-22 時点の表示。発表直前に再確認)
- リポジトリへの誘導: rust-dap / xous-core の該当ブランチ、QRコード等

---

## 準備タスク(発表までの TODO)

| # | タスク | 状態 | 備考 |
|---|---|---|---|
| 1 | GPIO 最大動作速度の数値確認 | ✅ | BIO トグルレート最大 25MHz(dabao-intro より) |
| 2 | Dabao 実機での GDB 疎通確認 | ✅ | 2026-08-22 E2E 全 PASS。スクショ撮影は残 |
| 3 | BIO オフロード実装 + 実測 | 実装済・入力側デバッグ中 | BIO 出力は動作確認済み。入力が読めない → 物理観測で切り分け |
| 4 | スライド化 | 未 | 本構成をベースに |
| 5 | デモ動画 or スクショ撮影 | 未 | 生デモは5分では危険 |

## 参照

- **開発者自身による Baochip / Dabao 紹介**(仕様・MMU 搭載の理由・BIO の説明はまずここ):
  <https://baochip.com/dabao-intro/>
- 『ハードウェアハッカー』(bunnie 著、技術評論社): <https://gihyo.jp/book/2018/978-4-297-10106-0>
- Xous(OS): <https://github.com/betrusted-io/xous-core/>
- Dabao ボード: <https://github.com/baochip/dabao> / 購入: <https://www.crowdsupply.com/baochip/dabao>
- BIO シミュレータ: <https://github.com/baochip/bio-sim>
- BIO ローダー: <https://github.com/baochip/bio-loader>
- BIO サーファー(波形ビューア): <https://baochip.com/bio-surfer/>
