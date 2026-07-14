# figure8_auto.lua コード解説

## 概要

**figure8_auto.lua** は、ArduPilot Copterを2つの接する円で構成した8の字軌道に沿って飛行させるLuaスクリプトです。

RC 6chをトリガーとして機体を **GUIDED** モードへ切り替え、開始地点を基準にした目標位置と目標速度を50 ms周期で生成し、**vehicle:set_target_posvel_NED()** へ送信します。

ミッションのウェイポイントを順番に通過する方式ではなく、Lua側で連続的に軌道を生成する方式です。

## このファイルの責務

- RC 6chによる8の字飛行の開始・停止判定
- 開始時のEKF origin基準NED位置の保存
- 接する2円からなる目標位置・目標速度の生成
- GUIDEDモードへの切り替え
- 停止時の元モード復帰
- 停止後の位相と経過時間のリセット

クラス定義はなく、関数とスクリプト内の状態変数で構成されています。

## 軌道の全体像

現在の8の字は、リサージュ曲線ではありません。開始地点を共通の接点として、NEDの東側と西側に同じ半径の円を1つずつ配置しています。

~~~text
                              North (+x)
                                   ^
                      .-----.      |      .-----.
                   .-'       '-.   |   .-'       '-.
West (-y) <-------( second circle )-o-( first circle )-------> East (+y)
                   '-.       .-'   |   '-.       .-'
                      '-----'      |      '-----'
                                   |
                                   o = start / contact point
~~~

- 第1円の中心: 開始地点から東へ **rad_xy_m**
- 第2円の中心: 開始地点から西へ **rad_xy_m**
- 両方の円の半径: **rad_xy_m**
- 2円の接点: 開始地点

## 設定値

### rad_xy_m

各円の半径です。現在値は **1.5 m** です。

この値では軌道範囲が次のようになります。

- North方向: -1.5 m から +1.5 m
- East方向: -3.0 m から +3.0 m
- 軌道全体: 約 3 m x 6 m

### target_speed_xy_mps

円周上の目標速度です。現在値は **1.0 m/s** です。

円軌道では、

~~~text
速度 = 半径 x 角速度
~~~

なので、角速度は次の式で求めています。

~~~lua
omega_radps = target_speed_xy_mps / rad_xy_m
~~~

リサージュ曲線だった旧実装と異なり、現在は設定値と円周上の速度が一致します。

### sampling_time_s

**update()** の呼び出し間隔として要求する時間です。現在値は **0.05 s**、つまり50 msです。

### ch6_threshold

RC 6chをONと判定するPWM閾値です。現在値は **1500** です。

## 主な状態変数

### theta

現在飛んでいる円の角度です。0から2piまで進み、1周すると2piを引いて0付近へ戻します。

### circle_direction

現在飛んでいる円が開始地点のどちら側にあるかを表します。

- **1**: 第1円。Y成分が正側
- **-1**: 第2円。Y成分が負側

**theta** が1周するたびに符号を反転するため、第1円と第2円を交互に飛びます。

### test_start_location

8の字を開始した地点です。

これは機体から見た相対位置ではなく、EKF originを原点とするNED絶対座標です。

### return_mode_num

8の字開始前のflight modeを保存します。停止時にこのモードへ戻します。

### figure8_active

8の字飛行を実行中かどうかを示します。

## 主要な関数

### restore_return_mode(reason)

開始前に保存したflight modeへ戻します。

1. **return_mode_num** が未設定なら何もしない
2. **vehicle:set_mode(return_mode_num)** を呼ぶ
3. 成功または失敗をGCSへ通知する
4. **return_mode_num** をクリアする

### set_start_location()

**ahrs:get_relative_position_NED_origin()** から現在位置を取得し、**test_start_location** に保存します。

保存する値の意味は次のとおりです。

~~~text
x: EKF originから北方向の距離 [m]
y: EKF originから東方向の距離 [m]
z: EKF originから下方向の距離 [m]
~~~

位置を取得できなければfalse、保存できればtrueを返します。

### figure8_target()

現在の **theta** と **circle_direction** から、開始地点を原点とする相対目標位置 **pos** とNED目標速度 **vel** を生成します。

処理は次の順番です。

1. 一定角速度 **omega_radps** で **theta** を進める
2. **theta** が2pi以上なら1周分を引く
3. 1周したときだけ **circle_direction** の符号を反転する
4. **senkai.lua** と同じ円軌道の位置と速度を計算する

### update()

スクリプトのメインループです。

RC入力、開始処理、軌道指令の送信、停止処理を担当し、最後に次回の実行間隔を返します。

## 第1円の数式

**circle_direction = 1** のときに第1円を描きます。

位置:

~~~text
x = r sin(theta)
y = r (1 - cos(theta))
z = 0
~~~

速度:

~~~text
vx = r w cos(theta)
vy = r w sin(theta)
vz = 0
~~~

ここで **r = rad_xy_m**、**w = omega_radps** です。この円の中心は相対座標 **(0, +r)** です。

## 第2円の数式

第1円を1周すると、次の処理で **theta** を0付近へ戻し、円の向きを反転します。

~~~lua
theta = theta - full_circle_rad
circle_direction = -circle_direction
~~~

**circle_direction = -1** のときはY成分の符号だけが反転し、第2円になります。

位置:

~~~text
x = r sin(theta)
y = -r (1 - cos(theta))
z = 0
~~~

速度:

~~~text
vx = r w cos(theta)
vy = -r w sin(theta)
vz = 0
~~~

この円の中心は相対座標 **(0, -r)** です。

## 2円の接続

第1円の終了点と第2円の開始点は、どちらも次の状態です。

~~~text
位置: (x, y) = (0, 0)
速度: (vx, vy) = (r w, 0)
~~~

そのため、接点で位置と速度は連続します。機体は接点で停止せず、そのまま第2円へ進みます。

ただし、2円は反対方向へ曲がるため、接点で旋回加速度の向きが反転します。位置と速度は連続ですが、加速度までは連続ではありません。

## 円周上の速度

速度の大きさは次のようになります。

~~~text
speed
= sqrt(vx^2 + vy^2)
= sqrt((rw cos)^2 + (rw sin)^2)
= rw
~~~

**w = target_speed_xy_mps / r** なので、

~~~text
speed = target_speed_xy_mps
~~~

です。現在の設定では全周で **1.0 m/s** になります。

## update() の1ループ

~~~mermaid
flowchart TD
    A["update()開始"] --> B["RC 6chを取得"]
    B --> C{"PWMを取得できたか"}
    C -- "No" --> D["1秒後に再実行"]
    C -- "Yes" --> E{"アーム済み かつ PWM > 1500"}
    E -- "Yes" --> F{"初回開始か"}
    F -- "Yes" --> G["現在モードと開始位置を保存"]
    G --> H["GUIDEDへ変更"]
    F -- "No" --> I["figure8_target()を計算"]
    H --> I
    I --> K["絶対位置と速度をGuidedへ送信"]
    K --> L["50 ms後に再実行"]
    E -- "No" --> M{"実行中だったか"}
    M -- "Yes" --> N["元モードへ復帰"]
    M -- "No" --> O["状態をリセット"]
    N --> O
    O --> L
~~~

### 開始時

1. RC 6chとアーム状態を確認する
2. 現在モードを **return_mode_num** に保存する
3. **set_start_location()** で開始地点を保存する
4. **vehicle:set_mode(4)** でGUIDEDへ変更する
5. **figure8_active = true** にする
6. 最初の目標位置・速度を送信する

### 実行中

1. **figure8_target()** で相対位置・速度を生成する
2. 相対位置へ **test_start_location** を加える
3. **vehicle:set_target_posvel_NED()** で送信する
4. 50 ms後に次のループを実行する

### 停止時

1. 元のflight modeへ戻す
2. **figure8_active = false** にする
3. 次回開始用に現在位置を取得する
4. **theta = 0**、**circle_direction = 1** に戻す

## 相対位置と絶対位置の合成

**figure8_target()** が返す **target_pos** は開始地点を原点とする相対位置です。

**update()** では次のように開始地点を加算します。

~~~lua
target_pos + test_start_location
~~~

~~~text
最終目標位置 = 開始時のNED絶対位置 + 8の字の相対位置
最終目標速度 = 8の字のNED速度
~~~

速度には開始位置を加えません。

## 座標系

このスクリプトが使う座標系はNEDです。

| 軸 | 正方向 |
|---|---|
| X | North |
| Y | East |
| Z | Down |

**figure8_target()** の相対位置もNED軸に沿っています。

機体Yawによる座標回転は行っていないため、8の字の向きは機首方向ではなく世界座標に固定されます。

## senkai.lua との関係

**senkai.lua** の円軌道は次の式です。

~~~text
x = r sin(theta)
y = r (1 - cos(theta))
~~~

**figure8_auto.lua** の第1円はこの式と同じです。第2円ではY成分の符号だけを反転し、反対側へ同じ円を配置しています。

つまり現在の実装は、**senkai.lua** の円を接点で2つ連結した構成です。

## 他ファイルとの依存関係

### ArduPilot Lua API

- Vector3f
- rc:get_pwm()
- arming:is_armed()
- ahrs:get_relative_position_NED_origin()
- vehicle:get_mode()
- vehicle:set_mode()
- vehicle:set_target_posvel_NED()
- gcs:send_text()

API定義:

- libraries/AP_Scripting/docs/docs.lua

### Copter Guided制御

**vehicle:set_target_posvel_NED()** の送信先です。

- ArduCopter/Copter.cpp
- ArduCopter/mode_guided.cpp

### AHRS / EKF

**get_relative_position_NED_origin()** の現在位置取得に関係します。

- libraries/AP_AHRS/AP_AHRS.cpp

## おすすめの読む順番

1. 冒頭の設定値で軌道サイズ、速度、周期を確認する
2. **update()** で開始・実行・停止の流れをつかむ
3. **set_start_location()** で開始地点の座標系を確認する
4. **figure8_target()** の第1円と第2円の切り替えを読む
5. **senkai.lua** と円の式を比較する
6. **Copter.cpp** と **mode_guided.cpp** で指令の送信先を確認する

## 注意点

- 2円の接点では位置と速度は連続するが、旋回加速度の向きは反転する
- **set_target_posvel_NED()** では加速度目標を明示的に送っていない
- **sampling_time_s** は要求周期であり、実際の経過時間を **millis()** で測定してはいない
- 目標送信失敗時はGCSへ通知するだけで、軌道生成自体は継続する
- 機体Yawを使っていないため、軌道の向きはNED基準で固定される
- 高度条件の確認は行っていないため、安全な高度への離陸は操縦者が行う必要がある
- 半径を小さくしたり速度を上げたりすると、必要な旋回加速度が増加する

円軌道で必要な向心加速度は次の式です。

~~~text
acceleration = speed^2 / radius
~~~

現在の **1.0 m/s**、半径 **1.5 m** では約 **0.67 m/s^2** です。

## 要約

**figure8_auto.lua** は、RC 6chを使ってCopterをGUIDEDへ切り替え、開始地点を接点とする2つの円を連続して飛ばすスクリプトです。

コードを理解するうえで重要なのは次の4点です。

- **figure8_target()** は開始地点基準の相対位置とNED速度を生成する
- 第1円と第2円はY成分の符号を反転して作る
- 接点では位置と速度が一致する
- **update()** が開始位置を加算して絶対NED目標として送信する
