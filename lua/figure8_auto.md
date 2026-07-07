# figure8_auto.lua 読解メモ

## 概要
`figure8_auto.lua` は、ArduPilot Copter 用の Lua スクリプトです。  
RC 6ch をトリガーにして機体を `GUIDED` モードへ切り替え、EKF origin 基準の `NED` 座標系で目標位置・目標速度を継続送信し、開始位置を基準とした 8 の字軌道を飛ばします。

このスクリプトは、あらかじめ用意されたミッションを実行するものではなく、Lua からリアルタイムに軌道を生成して Guided 制御へ流し込むタイプのスクリプトです。

## このファイルの責務
このファイルの責務は主に次の4つです。

- RC スイッチによる開始・停止判定
- 開始時の基準位置の保存
- 8 の字軌道の位置・速度生成
- `GUIDED` モードへの切り替えと解除時の元モード復帰

## 全体の流れ
1. スクリプト起動時に半径、目標速度、立ち上がり時間、サンプリング周期などを設定する
2. `update()` が一定周期で呼ばれる
3. RC 6ch の PWM が閾値を超え、かつ機体がアーム済みなら開始処理に入る
4. 開始時に現在位置を `test_start_location` として保存する
5. `GUIDED` モードへ切り替える
6. `circle()` で 8 の字軌道上の次の目標位置・目標速度を計算する
7. `vehicle:set_target_posvel_NED()` で Copter の Guided 制御へ送る
8. スイッチ OFF または非アーム状態になると元のモードへ戻し、内部状態をリセットする

## 主要な変数
- `rad_xy_m`
  - 8 の字のスケールを決める基本半径
- `target_speed_xy_mps`
  - 目標速度
- `ramp_up_time_s`
  - 最高速まで滑らかに立ち上げる時間
- `sampling_time_s`
  - 制御ループの周期
- `omega_radps`
  - 基本の進行速度係数
- `theta`
  - 軌道上の位相
- `time`
  - ランプアップ用の経過時間
- `test_start_location`
  - 開始時の絶対位置。EKF origin 基準の NED 座標
- `return_mode_num`
  - 開始前の flight mode
- `circle_active`
  - 実行中かどうかを表すフラグ

## 主要な関数
このファイルにクラス定義はありません。関数中心の構成です。

### `restore_return_mode(reason)`
開始前の flight mode を `return_mode_num` から復元します。  
スクリプトが `GUIDED` に切り替えたあと、停止時に元のモードへ戻すための後始末です。

役割:
- 元モードの復帰
- 復帰成功・失敗の GCS 通知
- `return_mode_num` のクリア

### `set_start_location()`
現在位置を `ahrs:get_relative_position_NED_origin()` から取得して、`test_start_location` に保存します。  
ここで保存しているのは、緯度経度ベースの `Location` ではなく、EKF origin 基準の `NED` 位置です。

役割:
- Guided の `set_target_posvel_NED()` にそのまま使える座標系で現在位置を取る
- 8 の字軌道の基準点を固定する

### `circle()`
名前は `circle` ですが、実際に作っているのは円ではなく 8 の字軌道です。  
位相 `theta` を少しずつ進めながら、その時点の位置 `pos` と速度 `vel` を返します。

役割:
- 軌道位相の更新
- 8 の字の相対位置生成
- その位置に対応する相対速度生成

### `update()`
メインループです。  
RC 入力の確認、開始処理、軌道指令の送信、停止処理、内部状態のリセットまで、この関数が担当します。

役割:
- RC 6ch の監視
- 開始条件判定
- `GUIDED` への遷移
- `circle()` による軌道生成
- `set_target_posvel_NED()` の送信
- 停止時の元モード復帰と変数リセット

## `update()` の1ループ
`update()` の1回の処理は大きく3パターンに分かれます。

### 1. 開始前または開始直後
- `rc:get_pwm(6)` を読む
- `arming:is_armed()` と PWM 閾値を確認する
- まだ `circle_active == false` なら開始処理に入る
- 現在モードを `return_mode_num` に保存する
- `set_start_location()` で開始位置を固定する
- `vehicle:set_mode(4)` で `GUIDED` に入る
- `circle_active = true` にする
- その同じループで `circle()` を呼び、最初の位置・速度指令を送る

### 2. 実行中
- 開始処理はスキップ
- `circle()` を呼び、現在の `theta` と `time` から相対位置・相対速度を作る
- `time` を `sampling_time_s` だけ進める
- `vehicle:set_target_posvel_NED(target_pos + test_start_location, target_vel)` を送る
- 50ms 後に次の `update()` が呼ばれる

### 3. 停止時
- RC スイッチ OFF または非アーム状態で停止側に入る
- `circle_active == true` なら `restore_return_mode()` を呼んで元モードへ戻す
- `circle_active = false`
- `set_start_location()` で現在地を次回開始用の基準に更新する
- `time = 0`, `theta = 0` に戻す

## `circle()` が作る 8 の字
`circle()` の本質は次の式です。

- `x = 2r * sin(theta)`
- `y = r * sin(2theta)`

ここで `r = rad_xy_m` です。  
`x` は 1周期で左右に1回振れ、`y` は 1周期で上下に2回振れるため、軌跡が 8 の字になります。

位置ベクトル:
- `pos.x = 2r * sin(theta)`
- `pos.y = r * sin(2theta)`
- `pos.z = 0`

速度ベクトル:
- `vel.x = 2r * cur_freq * cos(theta)`
- `vel.y = 2r * cur_freq * cos(2theta)`
- `vel.z = 0`

`cur_freq` は序盤だけ小さく、時間とともに増えます。  
そのため、開始直後はゆっくり、徐々に目標速度へ近づく滑らかな立ち上がりになります。

## `circle()` が返す `pos`, `vel` の意味
ここは読み違えやすいので重要です。

### `pos`
`circle()` が返す `pos` は、開始点を原点とした「相対位置」です。  
ただし軸の向きは機体前方基準ではなく、NED 基準です。

つまり:
- 機体前方に何 m
ではなく
- 北に何 m、東に何 m

を表しています。

### `vel`
`vel` は、その相対軌道をなぞるための速度ベクトルです。  
これも NED 基準です。

## `test_start_location` との合成
`update()` では、`circle()` が返した相対位置を開始時の絶対位置へ平行移動しています。

実際の送信は次の形です。

- `final_pos = test_start_location + target_pos`
- `final_vel = target_vel`

つまり、
- `target_pos` は「開始点からのずれ」
- `test_start_location` は「開始時の絶対位置」
- その和が「EKF origin 基準の絶対目標位置」

という関係です。

## 座標系の整理
このスクリプトでは次の3つを分けて考えると理解しやすいです。

### 1. EKF origin 基準の NED 絶対座標
例:
- `test_start_location`
- `set_target_posvel_NED()` に渡す最終位置

### 2. 開始点基準の相対座標
例:
- `circle()` が返す `pos`

### 3. 機体座標系
前・右・下のような機首基準の座標系です。  
このスクリプトでは使っていません。

## 機体座標系ではないことの意味
このスクリプトは yaw を使って座標回転していません。  
そのため、機体がどの向きで開始しても、8 の字の向きは機首基準では回らず、NED 基準で固定です。

たとえば:
- 北向きで開始しても
- 東向きで開始しても

描く 8 の字は「世界座標上で同じ向き」です。

## 他ファイルとの依存関係
このファイルに `require` はありませんが、実行時には ArduPilot の Lua API と Copter Guided 制御に依存しています。

### Lua API
- `Vector3f`
- `rc:get_pwm()`
- `ahrs:get_relative_position_NED_origin()`
- `vehicle:get_mode()`
- `vehicle:set_mode()`
- `vehicle:set_target_posvel_NED()`
- `gcs:send_text()`

Lua API の定義やドキュメント:
- `libraries/AP_Scripting/docs/docs.lua`

### Copter 側の Guided 実装
`vehicle:set_target_posvel_NED()` は Copter 本体の Guided 制御へつながっています。

主な関連箇所:
- `ArduCopter/Copter.cpp`
- `ArduCopter/mode_guided.cpp`

### AHRS / EKF 側
`ahrs:get_relative_position_NED_origin()` は、アクティブな EKF 実装から現在位置を取っています。

主な関連箇所:
- `libraries/AP_AHRS/AP_AHRS.cpp`

## 類似ファイルとの関係
### `libraries/AP_Scripting/examples/set_target_posvel_circle.lua`
公式の円軌道サンプルです。  
`figure8_auto.lua` はこの系統に近く、円ではなく 8 の字へ拡張し、開始位置取得とモード復帰を強化した形と読めます。

### `lua/archive/figure8_comp.lua`
こちらも 8 の字ですが、考え方が違います。  
`set_target_location()` で地点列を順番に踏ませる離散型で、`start_yaw` を使って前後左右を北東成分へ回しています。

対して `figure8_auto.lua` は:
- 連続的に位置・速度を生成する
- NED 基準で毎ループ指令を送る

という違いがあります。

## おすすめの読む順番
1. 設定値と状態変数を見る
2. `update()` を読んで、開始条件・停止条件・全体制御をつかむ
3. `set_start_location()` を読んで、なぜ `Location` ではなく EKF origin/NED を使うか理解する
4. `circle()` を読んで、8 の字の数式と速度生成を見る
5. `set_target_posvel_circle.lua` を読んで元ネタとの差分を見る
6. `ArduCopter/Copter.cpp` と `mode_guided.cpp` を読んで、Lua から送った指令が機体本体でどう処理されるか確認する

## 読むときに注目するとよい変数
- `circle_active`
  - 実行中かどうか
- `return_mode_num`
  - 元モードの退避先
- `test_start_location`
  - 8 の字の原点
- `time`
  - ランプアップ用の時刻
- `theta`
  - 軌道上の位相

## 注意点
- 関数名 `circle()` とは裏腹に、実際は円ではなく 8 の字軌道
- 冒頭コメントには円軌道の名残があり、そのまま信じると誤読しやすい
- `HOVER_ALT_CM` は現状コード上で使われていない
- `GUIDED` 前提なので、Copter 側が Guided 制御可能な状態でないと動作しない
- 位置が取れない場合は `set_start_location()` が失敗し、開始できない

## 要約
`figure8_auto.lua` は、Copter を RC スイッチで `GUIDED` に切り替え、開始地点を基準にした 8 の字の相対軌道を EKF origin 基準の絶対 NED 座標へ変換して送り続ける Lua スクリプトです。

処理の中心は `update()` で、軌道生成の中心は `circle()` です。  
理解のポイントは、
- `pos` は開始点基準の相対位置
- `test_start_location` は開始時の絶対位置
- `set_target_posvel_NED()` に渡すのは絶対 NED 位置と NED 速度
という3点です。
