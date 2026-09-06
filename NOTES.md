# LEDシーリングライト BLE制御 調査ログ (2026-09-03)

## 対象デバイス
- Amazon: https://www.amazon.co.jp/dp/B0FQMZMPH1
- 操作アプリ: **LampSmart Pro** (developer: XuRenNan, package: `com.jingyuan.lamp`)
- 制御方式: BLEの生アドバタイジングパケットへの一方向ブロードキャスト(ペアリング/GATT接続なし、iBeaconに近い)
- 実機のBluetooth MAC(リモコン側と思われる): `10:9E:3A:10:25:5D`

## 結論: 動いたプロトコル

`MasterDevX/lampify` (https://github.com/MasterDevX/lampify) と
`zt8989/esphome-lampsmart-pro` (https://github.com/zt8989/esphome-lampsmart-pro)
が実装している「CRC16 + LFSRホワイトニング」方式の**プロトコルはこのランプでは動かなかった**。

代わりに `aronsky/esphome-components` の `ble_adv_controller` コンポーネント
(https://github.com/aronsky/esphome-components/tree/main/components/ble_adv_controller)
にある **`FanLampEncoderV2`** クラスの **`other` / `v3` バリアント**が実機で動作した。

- 実装: https://github.com/aronsky/esphome-components/blob/main/components/ble_adv_controller/fanlamp_pro.cpp
- 設定値(config): https://github.com/aronsky/esphome-components/blob/main/components/ble_adv_controller/__init__.py
  内の `"other"` → `"v3"`:
  ```python
  "v3": {
      "class": FanLampEncoderV2,
      "args": [ [0x10, 0x80, 0x00], 0x0100, True ],  # prefix, device_type, with_sign
      "ble_param": [ 0x19, 0x16 ],
      "header": [0xF0, 0x08],
  },
  ```
- ソースコード中のコメントに「FanLamp Pro / LampSmart Proアプリが実際に生成するものとは一致しない。おそらく一部のリモコンが生成しているもの」とあり、まさに今回捕まえたのが物理リモコンのトラフィックだったことと符合する。

### なぜ最初のlampify系プロトコルが動かなかったか

nRF ConnectとESP32自前のBLEスキャナで実機のパケットをキャプチャして分かった相違点:

1. AD type が `0x16` (Service Data - 16bit UUID) であって、lampify.c が使う `0x03` (Service UUID List) ではなかった
2. `Connectable: No` (ADV_NONCONN_IND) であって、lampify.cのデフォルト値 `ADV_IND` (connectable) ではなかった
3. ペイロード内の可変領域が単純なCRC16+LFSRホワイトニングでは説明できないランダム性を持っていた
   → 実際にはAES-128-ECBによる16bit署名("sign")が乗っており、ホワイトニングも128バイトの`XBOXES`テーブル参照方式(LFSRではない)だった

## パケット構造 (FanLampEncoderV2, other/v3)

31バイトの生アドバタイジングデータ(AD payload):

```
[0..2]   02 01 02              -- Flags AD構造 (len=2, type=0x01, value=0x02)
[3]      0x1B                  -- 次のAD構造の長さ (27 = 1(type) + 2(header) + 24(buf))
[4]      0x16                  -- AD type: Service Data - 16bit UUID
[5..6]   F0 08                 -- header (固定, "UUID"として見える部分)
[7..30]  buf[0..23]            -- 24バイトの本体 (下記)
```

`buf` (24バイト、offsetは buf 内での相対位置):

| offset | サイズ | 内容 |
|---|---|---|
| 0-2 | 3 | prefix固定値 `10 80 00` |
| 3 | 1 | tx_count (送信ごとにインクリメントするカウンタ) |
| 4-5 | 2 (LE) | device_type = `0x0100` |
| 6-9 | 4 (LE) | identifier (機器固有ID。今回はMACから4バイト取って使用) |
| 10 | 1 | group_index (今回は常に0) |
| 11-12 | 2 (LE) | command (下記コマンド表) |
| 13-16 | 4 | args (コマンドにより意味が変わる) |
| 17-18 | 2 (LE) | sign (AES署名の下位16bit、後述) |
| 19 | 1 | spare (常に0) |
| 20-21 | 2 (LE) | seed (乱数、パケットごとに新規生成) |
| 22-23 | 2 (LE) | crc16 |

### 既知のコマンドコード

| コマンド | 値 | args | 備考 |
|---|---|---|---|
| PAIR | 0x28 | 0,0,0,0 | ペアリング。ランプ電源投入後5秒以内に送ると点滅して成功 |
| UNPAIR | 0x45 | 0,0,0,0 | |
| LIGHT_ON | 0x10 | 0,0,0,0 | |
| LIGHT_OFF | 0x11 | 0,0,0,0 | |
| LIGHT_WCOLOR (dim) | 0x21 | args[2]=cold(0-255), args[3]=warm(0-255) | 色温度と明るさを同時制御。0-255の直接値(lampify系の10段階テーブルではない) |
| **常夜灯トグル** | **0x23** | 0,0,0,0 | リモコンの月アイコンボタンを実キャプチャ+自前デコーダで解析して発見。`LIGHT_SEC_ON/OFF`(0x12/0x13、aronsky実装のCommandType enum上の想定値)は不正解で反応なし。**on/offと違いトグル式(冪等でない)なので、複数回連続送信すると点滅・多重トグルしてしまう**。1回だけ送るのが安全だが、非connectable broadcastで到達保証がないため、確実性と要調整(今日は未解決) |
| タイマー(15分/30分) | 不明 | - | `CommandType` enumに対応する型が存在しない。リモコン側がローカルでタイマーを持って後で`off`を送っているだけの可能性が高い。今回は「使わない」とのことで未調査 |

### ホワイトニング (XBOXES方式)

```c
static const uint8_t XBOXES[128] = { ... }; // main.c参照

void whiten(uint8_t *buf, int size, uint8_t seed, uint8_t salt) {
    for (int i = 0; i < size; i++) {
        buf[i] ^= XBOXES[((seed + i + 9) & 0x1f) + (salt & 0x3) * 0x20];
        buf[i] ^= seed;
    }
}
```

`buf[2..19]` (18バイト) にのみ適用。`buf[0..1]`(prefixの一部)と `buf[20..23]`(seed, crc16)は平文のまま。
**位置ごとの自己逆変換**なので、seed(平文で埋め込まれている)さえ分かれば同じ関数をもう一度かけるだけで復号できる。今回はこれを使って月アイコンボタンの実コマンドを直接読み取った。

### AES署名 (sign)

```c
uint8_t sigkey[16] = {0,0,0, 0x0D,0xBF,0xE6,0x42,0x68,0x41,0x99,0x2D,0x0F,0xB0,0x54,0xBB,0x16};
sigkey[0] = seed & 0xFF;
sigkey[1] = (seed >> 8) & 0xFF;
sigkey[2] = tx_count;
// AES-128-ECB (ESP-IDF/mbedtlsの mbedtls_aes_setkey_enc + mbedtls_aes_crypt_ecb)
// buf[1..16] (16バイト、prefixの後半2バイト+tx_count~args、ホワイトニング前の平文)を暗号化
// 出力の先頭2バイト(リトルエンディアン)が sign。0なら0xFFFFに置換
```

`sign`計算は**ホワイトニングをかける前**に行う。計算順序: フィールド埋め → sign計算 → ホワイトニング → CRC計算、の順(CRCは既にホワイトニングされたバイト列に対して計算する)。

### CRC16

lampify.c由来のテーブル(poly 0x1021, MSB-first, 反転なし)をそのまま流用可能。ただし初期値が固定`0xFFFF`ではなく `~seed` になる点がlampify版と異なる。esphomeでは `esphome::crc16be(buf, len, ~seed)` として使われている。

## 実装

`~/codes/misk/light/main/main.c` — ESP-IDF(PlatformIO)ネイティブプロジェクト、ESP32-C3向け。
コンソール(USB CDC経由、`pio device monitor`または生シリアルで操作)からコマンドを打てる:

```
pair / unpair / on / off / cold <0-9> / warm <0-9> / dual <0-9> / full / half / night / scan [sec] / replay
```

- `scan [seconds]`: BLEスキャンして `16 F0 08` シグネチャに一致するパケットのみ生バイトでログ表示。実機解析用。
- `replay`: 過去にキャプチャした実パケットをそのまま再送(デバッグ用)。

### ビルド/書き込み

```sh
cd ~/codes/misk/light
./pio-venv/bin/pio run
./pio-venv/bin/pio run -t upload --upload-port /dev/cu.usbmodem1101
```

ESP32-C3の実機ポートは `/dev/cu.usbmodem1101`(ネイティブUSB、USB-Serial/JTAG)。
`sdkconfig.defaults` で以下がポイント:
- BLE 4.2レガシーAPI有効化(`esp_ble_gap_config_adv_data_raw`等を使うため。デフォルトはBLE5.0拡張adv専用でこれらの関数がリンクされない)
- console を native USB Serial/JTAG に切り替え(デフォルトのUART0は物理ピンが未結線で標準入力が届かない)
- mainタスクのスタックサイズ拡張(BLE初期化+USB-CDCコンソールドライバで純正3584Bでは足りずクラッシュする)

## 未解決・今後の課題

1. **常夜灯トグル(0x23)の信頼性**: non-connectable broadcastで到達保証がない一方、トグルは冪等でないので連打できない。実装するなら「送信後に一定時間反応がなければ確認を促す」等のUXか、複数回に分けて送りつつ受信側の重複排除ロジック(あれば)を頼る設計が必要
2. **タイマー(15分/30分)ボタン相当**: 未解析。必要になったら同じ手法(スキャン+自前デコーダ)で捕まえられるはず
3. 本実装(ESPHome化 or 常用ファームウェア化)は未着手。今回はコンソール経由の検証止まり

## 外部(Mac/Home Assistant)からの操作方式について

一番楽なのは**ESPHome化してHome Assistant経由**にすること。今日見つけた
`aronsky/esphome-components` の `ble_adv_controller` はまさにこの `other`/`v3` バリアントを
標準サポートしているので、自前Cコードを本実装として保守しなくて済む。HA経由ならMacからは
既存のHA REST API/自動化で叩けるし、HA無しでも `aioesphomeapi` で直接ESP32のnative APIを
叩ける。Wi-Fi設定が必要になる分、今日避けた「Wi-Fi要らない」制約とはトレードオフ。

ただし注意点として、**常夜灯トグル(0x23)は`aronsky/esphome-components`の`CommandType` enumに
存在しないコマンド**(標準では `LIGHT_SEC_ON`/`LIGHT_SEC_OFF` = 0x12/0x13が対応するはずだが、
実機では反応せず0x23が正解だった)。つまりコンポーネントをそのまま使うだけでは常夜灯は
カバーできず、以下のどれかが必要になる:
- コンポーネント側にカスタムコマンド送信の口(生コマンドバイトを指定できるAPI)があればそれを使う
- なければコンポーネントをフォークして`CommandType`に追加する
- 最悪、常夜灯だけ諦めるか、別経路(自前の小さな追加ファームウェア/BLE送信)で面倒を見る

**ローカルパッチは可能**: ESPHomeの`external_components`は`type: local, path: <ローカルパス>`で
GitHubではなくローカルディレクトリを直接参照できる。`aronsky/esphome-components`をリポジトリに
コピーして`ble_adv_handler.h`の`CommandType` enumに`NIGHT_TOGGLE = 0x23`相当を足し、
`fanlamp_pro.cpp`の`translate()`にマッピングを追加すればいい。GitHub上でforkする必要すらない。

MQTT+自前HTTPサーバでHA無しの最小構成にする案もあったが、その場合は今日の資産
(aronskyコンポーネント)を使わず自分で全部保守することになるので非推奨。

## 参考URL

- https://github.com/MasterDevX/lampify — 最初に参照した簡易プロトコル実装(このランプでは不一致)
- https://github.com/zt8989/esphome-lampsmart-pro — 同上のESPHome版(同じく不一致)
- https://github.com/aronsky/esphome-components — 今回動いた実装の本体
  - https://github.com/aronsky/esphome-components/blob/main/components/ble_adv_controller/fanlamp_pro.cpp — FanLampEncoderV2の実装
  - https://github.com/aronsky/esphome-components/blob/main/components/ble_adv_controller/__init__.py — バリアント設定一覧
  - https://github.com/aronsky/esphome-components/blob/main/components/ble_adv_controller/README.md — ESPHome側のYAML設定方法
- https://www.amazon.co.jp/dp/B0FQMZMPH1 — 対象製品
