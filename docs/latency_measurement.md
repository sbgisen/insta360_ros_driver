# X3カメラ 遅延実測結果とレイテンシ計測モード

> **一時的な計測モードについて**
> ここで説明する `enable_latency_debug_log` 計測モード（`OnVideoData`/`OnExposureData`
> コールバック内のCSVロギング）は、Phase 2.5 の `header.stamp` 補正実装に向けた一時的な
> instrumentationであり、実測が完了し補正方式が確定した段階で `src/main.cpp` から
> revertされる可能性がある。一方、本ドキュメントに記載する**実測結果および
> `GetCameraMediaTime()` に関する制約知見はコードのrevert後も独立して有効**であり、
> 今後 `GetCameraMediaTime()` の利用や同等の遅延計測を再検討する際の参照情報として
> 本ファイルに残す。

## 背景

Insta360 X3 のライブストリーム映像に付与する ROS メッセージの `header.stamp` を、
カメラSDK側のタイムスタンプに基づいて補正するため（Phase 2.5）、実際のフレーム配信の
ジッタやオフセットの性質を実機で計測した。以下は Scanner 実機上での実測結果（約13.2分間、
23,702フレーム、X3 dual-fisheye live stream）のまとめである。

## 重要な制約: `GetCameraMediaTime()` はlive stream中に呼んではならない

`OnVideoData`（カメラSDKの動画データコールバックスレッド）内で `cam_->GetCameraMediaTime()`
を毎フレーム同期的に呼び出すと、**呼び出した時点でカメラの内部コマンドチャンネルが応答不能
（`failed to get response message from camera because of time out`）になり、video stream
が停止する**ことを実機検証で確認した。カメラの電源再投入を挟んで3回試行し、いずれも
100%再現したため、ハードウェア起因ではなくコード起因（SDKの内部制約）と判断している。

この呼び出しを完全に無効化（`media_time` を常に `-1.0` として扱う）したところ、13分以上
安定して連続ストリーミングでき、フレームドロップも発生しなかった。

**今後 `GetCameraMediaTime()` を使う場合は、`OnVideoData` コールバックスレッドから直接・
同期的に呼び出してはならない。**別スレッドやタイマーコールバックなど、動画データコール
バックの実行を阻害しない形に非同期化することが必須である。

## 実測条件と主要結果

- 対象: Insta360 X3、dual-fisheye live stream（`RES_3840_1920P20` 系）、USB接続
- 計測時間: 約13.2分間、23,702フレームを連続収集
- `GetCameraMediaTime()` は上記の理由により無効化した状態での計測

| 項目 | 結果 |
|---|---|
| frame-to-frame jitter（平均） | 33.37 ms（≈29.97 fps） |
| frame-to-frame jitter（標準偏差） | 0.48 ms |
| フレーム間隔の最小/最大 | 33 ms / 34 ms（この2値のみ） |
| 100ms超のギャップ（フレーム欠落疑い） | 0件 / 23,701 |
| 13.2分を10等分したチャンク間の平均・標準偏差 | 全チャンクで完全一致（ドリフト未検出） |
| `OnExposureData` の発火率 | 23,702フレーム中22,551個がユニーク値（約95%でほぼ毎フレーム発火） |
| `exposure_timestamp - sdk_timestamp` | 平均 +81.8 ms、標準偏差 11.1 ms（範囲 66〜132 ms） |

これらの結果から、**「固定offset + 小jitter」モデルが実用上妥当**という結論に至った。
起動直後に1回オフセットをキャリブレーションし、以降はそのオフセットを使い続ける方式が
現実的と判断できる（13分の実測範囲でドリフトは検出されなかったが、数時間規模の長時間運用
でのドリフト有無は今回の実測範囲では未検証）。

`OnExposureData` もほぼ毎フレーム発火し、`sdk_timestamp` との差が安定していることから、
露光時刻ベースの補正（`sdk_timestamp` ベースの代替・併用案）も選択肢になり得る。

## `enable_latency_debug_log` 計測モードの使い方

`enable_latency_debug_log` パラメータを有効にすると、`OnVideoData` / `OnExposureData`
コールバック内で取得したタイムスタンプ情報をCSV形式でファイルに記録できる。

現時点では `bringup.launch.py` に専用の launch 引数は用意されていないため、ノード実行時に
`--ros-args -p` で直接パラメータを渡す。

```bash
ros2 run insta360_ros_driver insta360_ros_driver --ros-args \
  -p enable_latency_debug_log:=true \
  -p latency_debug_log_path:=/tmp/insta360_latency_debug.csv
```

- `latency_debug_log_path` を省略した場合のデフォルト出力先: `/tmp/insta360_latency_debug.csv`
- デフォルトでは `enable_latency_debug_log` は `false` であり、無効時はロギング処理が一切
  実行されないため本番動作に影響しない。

**実測時の注意点**: このモードはあくまでロギングのみを行うものであり、
`GetCameraMediaTime()` 自体は呼び出さない（`media_time` 列は常に `-1.0` を記録する。
理由は後述）。上記の「重要な制約」に反してこの計測コード自体を改変し
`GetCameraMediaTime()` の呼び出しを追加することは避けること。

## CSVフォーマット

出力CSVのヘッダーと各列の意味は以下の通り。

| 列名 | 意味 |
|---|---|
| `frame_seq` | `OnVideoData` 呼び出しごとに0から連番で増加するフレーム通し番号 |
| `sdk_timestamp` | `OnVideoData` に渡される `timestamp`（カメラSDK内部のミリ秒スケールの単調カウンタ。起動時0近辺から増加し、Unix epochではない） |
| `exposure_timestamp` | 直近の `OnExposureData` コールバックでキャッシュされた露光タイムスタンプ。`OnVideoData` と `OnExposureData` は非同期の別コールバックであり厳密なペアリングではなく近似値である点に注意 |
| `media_time` | 常に `-1.0` 固定。理由は次節参照 |
| `now_stamp_sec` | `OnVideoData` 呼び出し時点でのROSノードクロック（`node_->get_clock()->now()`）の秒値。ROSメッセージの `header.stamp` に設定される値と同じ |

### `media_time` が `-1.0` 固定である理由

前述の通り `GetCameraMediaTime()` はlive stream中に呼ぶとストリームが停止するため、
このロギングコードは `GetCameraMediaTime()` を一切呼び出さず、`media_time` 列には常に
`-1.0` を記録する（既存のCSVパーサとの互換性のため列自体は残している）。

## Phase 2.5 実装への提案

1. 本番の `header.stamp` 補正案として、ライブストリーム開始直後に `sdk_timestamp`
   （または `exposure_timestamp`）と `now()` の対応点を数フレーム分収集して固定offsetを
   キャリブレーションし、以降は `now() = sdk_timestamp / 1000 + offset` で補正する方式が
   有力候補。
2. `OnExposureData` ベースの露光時刻補正も、案1の代替または併用として検討の余地がある。
3. `GetCameraMediaTime()` を利用する場合は、`OnVideoData` コールバックスレッドから
   直接・同期的に呼び出さず、非同期化（別スレッド/タイマーコールバック等）した上で
   別途実測・検証すること。
4. 長時間運用（数時間〜）でのドリフト再検証は、今回の13分間の実測範囲では行えていない
   limitationとして残っている。次回実測時に確認することが望ましい。
