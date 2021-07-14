# A CMSIS-DAP implementation in Rust

## 概要

Arm用のデバッグ・アダプタのプロトコルおよびファームウェアの規格であるCMSIS-DAPのRust実装です。

現時点では他の実装と比較して性能やデバッグ・アダプタとしての機能面での優位性はありませんが、正しいWCID (Windows Compatibility ID) を返すことにより、Windowsでドライバの手動インストールを行わずに使用できます。

現在の所、動作するデバイスは `Seeeduino XIAO` のみとなっています。

## ライセンス

ライセンスは `Apache-2.0 License` に従います。詳しくは ![LICENSE](./LICENSE) ファイルを確認してください。