led.luaは、RC送信機のスイッチ状態を監視し、その状態に応じてNeoPixel LEDの色を変更するプログラムです。
local *FIGURE_8_CH    = 9   -- 【優先度2】8の字飛行スイッチ `8の字飛行をするときは9チャンネル`
local *AUTO_FLIGHT_CH   = 7   -- 自動離陸スイッチ`自動離陸するときは7チャンネル`

local LED_SERVO_CH   = 9   -- LEDが繋がっている`LEDを制御するのは9チャンネル`
local NUM_LEDS       = 16   -- 繋がっているLEDの数`LEDは16個あります`

-- LEDの初期化
*serialLED:set_num_neopixel(LED_SERVO_CH, NUM_LEDS)`LEDドライバへ出力は9番でLEDは16個あると伝えています`

-- LEDの色をまとめて変更する関数
local *function set_led_color(r, g, b)`set_led_cplorという関数があるよ`
    for i = 0, NUM_LEDS - 1 do`0～15番まで順番に処理を行うよう`
        *serialLED:set_RGB(LED_SERVO_CH, i, r, g, b)`番号iのLEDの色を(r,g,b)に設定します`
    end`15番まで処理が終わった`
    *serialLED:send(LED_SERVO_CH) -- 送信して光らせる`色を設定してLEDへまとめて送信します`
end　

function update()`ArduPilotが繰り返し呼び出す関数。100msごとに実行されます`
    -- 各チャンネルのPWM値を読み取る
    local fig8_pwm  = rc:get_pwm(FIGURE_8_CH)`RC9chのPWM値を取得します`
    local auto_pwm = rc:get_pwm(AUTO_FLIGHT_CH)`RC7chのPWM値を取得します`
    -- 値が1つでも取れなければ何もしない(すべてのスイッチの接続確認)
    if fig8_pwm or not auto_pwm then `もしfig8_pwmが取得できないまたはauto_pwmが取得できないなら`
        return update, 100`100ms後にもう一度呼び出します`
    end`if文の終わり`

    ----------------------
    -- if, elseif を使って、優先順位を決めて判定
    -- 【優先度2】8の字飛行 (青 / Blue)
    if fig8_pwm > 1800 then
        set_led_color(0, 0, 50)`もし9chが1800以上ならLEDを青色にします`
        
    -- 【優先度3】自動離陸 (紫 / Purple)
    *elseif auto_pwm > 1800 then
        set_led_color(50, 0, 50)`もし7chが1800以上ならLEDを紫色にします`
        
    -- 【ハンズオフ飛行中以外 (すべてのミッションがOFFの時)】
    else
        -- 通常状態の赤色 (Red)
        set_led_color(50, 0, 0)`赤色になります`
    end

    -- 100ミリ秒(0.1秒)後にまた色をチェックする
    return update, 100`100ms後にもう一度update()を実行してくださいという意味です。これによって0.1秒ごとにスイッチを監視しています`
end

gcs:send_text(6, "LED Mode Switch Script Loaded")`Mission PlannerなどのGCSへLED Mode Switch Script Loadedという文字を表示します`
return update()`update()を一度実行します`


*
FIGURE_8_CH --8の字飛行を表す変数の名前
AUTO_FLIGHT_CH--自動離陸を表す変数の名前
function set_led_color(r, g, b)--set_led_colorという名前の関数で引数(r, g, b)を持ちます
serialLED:set_num_neopixel(...)--serialLEDはArduPilotが用意しているAPIです。Luaでは:はオブジェクトの関数を呼び出すという意味です。ArduPilotのLuaでは
rc:get_pwm()
gcs:send_text()
serialLED:send()
などの書き方があります。
serialLED:send(LED_SERVO_CH)--serialLEDという機能の中にあるsendという関数を実行します

local function set_led_color(r, g, b)
    for i = 0, NUM_LEDS - 1 do
        serialLED:set_RGB(LED_SERVO_CH, i, r, g, b)
    end
    serialLED:send(LED_SERVO_CH) -- 送信して光らせる
end　
--functionは関数です。関数とは何度も使う処理をまとめたものです。例えばLED16個全部を(r,g,b)色にするという処理を毎回書くと
    for i = 0, NUM_LEDS - 1 do
        serialLED:set_RGB(LED_SERVO_CH, i, r, g, b)
    end
    serialLED:send(LED_SERVO_CH) 
を何度も書くことになります。そこでset_led_color()という名前の関数を作っています。すると全てのLEDを赤色にしたいときはset_led_color(50,0,0)と書くだけで済みます。

RCスイッチ | LEDの色 | 条件
8の字飛行  | 青   　 | fig8_pwm > 1800  
自動離陸   | 紫    　| auto_pwm > 1800 fig8<=1800
それ以外   | 赤    　| auto_pwm <=1800 fig8<=1800


文法               | このコードでの例               |              
 `local`           | `local NUM_LEDS = 16`        | ローカル変数の前に書く                 
 `function`        | `function update()`          | 関数の前に書く                 
 引数              | `set_led_color(r,g,b)`       | 関数に(r,g,b)を渡す                       
 `for`             | `for i = 0, NUM_LEDS-1 do`   | i=0から15まで同じ処理を繰り返す                     
 `if`              | `if fig8_pwm > 1800 then`    | 条件によって処理を分ける                  
 `elseif`          | `elseif auto_pwm > 1800 then`| 2つ目以降の条件を判定する                 
 `else`            | `else`                       | どの条件にも当てはまらない場合の処理            
 `end`             | `end`                        | `if`・`for`・`function` の終わりを示す 
 `:`               | `rc:get_pwm()`               | ArduPilot APIのメソッドを呼び出す       
`return update,100`| `return update,100`          | 100ms後に関数を再実行する                