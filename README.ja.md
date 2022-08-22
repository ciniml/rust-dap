# A CMSIS-DAP implementation in Rust

[English](./README.md) 日本語
## 概要

![デバッグボード](./doc/figure/debug_board.drawio.svg)

Arm用のデバッグ・アダプタのプロトコルおよびファームウェアの規格であるCMSIS-DAPのRust実装です。

現時点では他の実装と比較して性能やデバッグ・アダプタとしての機能面での優位性はありませんが、正しいWCID (Windows Compatibility ID) を返すことにより、Windowsでドライバの手動インストールを行わずに使用できます。

## 対応ボード

現在対応しているボードは以下の通りです。
各ボードごとのの実装については、 [boards](./boards) ディレクトリ以下に含まれています。

| ボード名           | 対応機能         | ディレクトリ         |
|:------------------|:----------------|:--------------------|
| Seeeduino XIAO    | CMSIS-DAP       | [./boards/xiao_m0](./boards/xiao_m0) | 
| XIAO RP2040       | CMSIS-DAP, UART | [./boards/xiao_rp2040](./boards/xiao_rp2040) | 
| Raspberry Pi Pico | CMSIS-DAP, UART | [./boards/rpi_pico](./boards/rpi_pico) | 

## ライセンス

ライセンスは `Apache-2.0 License` に従います。詳しくは [LICENSE](./LICENSE) ファイルを確認してください。