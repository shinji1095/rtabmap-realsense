# 概要
このドキュメントではWindowsに接続したUSBデバイスをWSLで認識するための手順である．特に，V4L2ドライバを経由して利用するデバイス（例えば，Realsenseシリーズなど）では，Linuxカーネルの設定を変更する必要がある．この手順書ではその過程を説明する．

# 目次

- [概要](#概要)
- [目次](#目次)
- [1. USBIPD-WINのインストール](#1-usbipd-winのインストール)
- [2. USBデバイスを接続](#2-usbデバイスを接続)
- [📖参考文献](#参考文献)


# 1. USBIPD-WINのインストール

1. Linuxカーネルバージョンの確認

Linuxカーネルのバージョンが`5.10.60.1`以上である必要がある．

以下のコマンドを実行しカーネルバージョンを確認する．
```shell
$ uname -r

-----------出力-------------------
5.15.146.1-microsoft-standard-WSL2
----------------------------------
```
2. usbipd-winのインストール

[usbipd-win](https://github.com/dorssel/usbipd-win/wiki/WSL-support/6befeedd4c8e2a49468e4b03532c9a20478f8677)にアクセスしインストーラ経由でusbipd-winをインストールする．

# 2. USBデバイスを接続

TBD

# 📖参考文献
- [USB デバイスを接続する](https://learn.microsoft.com/ja-jp/windows/wsl/connect-usb)