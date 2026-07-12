# 自作USBデバイスにWinUSBドライバを自動インストールする

## 概要

Windowsの場合、HIDやCDCといったUSBの規格で定められているクラスの場合はOS標準のデバイスドライバが自動でインストールされます。
一方、ベンダ固有クラスなどの場合、最近では専用のデバイスドライバを使用せずに `WinUSB` 等の汎用ドライバを使用することがあります。

このとき、WinUSBといったWindows標準ドライバの場合でも、ドライバパッケージを作成するか、`Zadig` といったツールを使ってドライバのインストールが必要です。

正式なドライバパッケージを作成するには、署名用の証明書を購入してドライバパッケージへの署名が必要となります。
また、Zadigは便利でありますが、Zadigをつかってドライバのインストールを行う手順が余分に発生します。

そこで、Windowsに用意されているWinUSBドライバを自動インストールする仕組み `WCID` を使えるようにします。

## WCID (Windows Compatibility ID)

WindowsにはそのデバイスがWinUSBなどの汎用ドライバと互換性があるかどうかを表すための WCID という仕組みがあります。
USBデバイスは、いくつかの特殊なディスクリプタをMicrosoftが定めた規格に則って返すことにより、デバイスやインターフェースに対する WCID を指定することができます。

WCIDを指定するためのディスクリプタの内容は、 [Microsoft OS 2.0 Descriptor](https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/microsoft-os-2-0-descriptors-specification) で規定されています。

## WCIDを指定するディスクリプタの構成

WCIDを指定するディスクリプタの構成は次のとおりです。

1. デバイス・ディスクリプタの `bcdUSB` に `0x0210` (USB 2.1) 以上を指定して、デバイスが USB 2.1の規格に従うことを指定します。これは後述する `BOSディスクリプタ` を使用するために必要です。
2. `GET_BOS_DESCRIPTOR` の要求に対してBOSディスクリプタを返すようにします。返すBOSディスクリプタには、`Microsoft OS 2.0 Platform Capability Descriptor` が含みます。
3. `bmRequestType == Vendor` && `wValue == (Microsoft OS 2.0 Platform Capability Descriptor)` &&  `Value == 7` && `wIndex == 0` のControl IN要求に対して、 `MS OS 2.0 Descriptor Set` のディスクリプタを返します。

### BOSディスクリプタ

USB2.1以降、様々なバイナリデータを保持するディスクリプタとして、BOS (Binary Object Store) ディスクリプタが定義されました。
BOSディスクリプタのディスクリプタ番号は `15` で以下の構造を持ちます。

| フィールド名 | 長さ | 内容 |
|:-------------|----:|:------|
| bLength                | 1            | ディスクリプタの全体の長さ                            |
| bDescriptorType        | 1            | ディスクリプタの種類。 DEVICE CAPABILITYディスクリプタ |
| bDevCapabilityType     | 1            | PLATFORM |
| bRserved               | 1            | 0 |
| PlatformCapabilityUUID | 16           |   |
| CapabilityData         | bLength - 20 |   |

