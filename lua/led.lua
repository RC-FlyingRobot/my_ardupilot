-- 各ミッションに割り当てるプロポのチャンネル番号 (環境に合わせて変更)
local MOTOR_FAIL_CH  = 6   -- 【優先度1】耐故障（モーター停止）テスト用スイッチ
local FIGURE_8_CH    = 7   -- 【優先度2】8の字飛行スイッチ

local LED_SERVO_CH   = 5   -- LEDが繋がっているPWM出力ピン
local NUM_LEDS       = 16   -- 繋がっているLEDの数

-- LEDの初期化
serialLED:set_num_neopixel(LED_SERVO_CH, NUM_LEDS)

-- LEDの色をまとめて変更する関数
local function set_led_color(r, g, b)
    for i = 0, NUM_LEDS - 1 do
        serialLED:set_RGB(LED_SERVO_CH, i, r, g, b)
    end
    serialLED:send(LED_SERVO_CH) -- 送信して光らせる
end

function update()
    -- 各チャンネルのPWM値を読み取る
    local fail_pwm  = rc:get_pwm(MOTOR_FAIL_CH)
    local fig8_pwm  = rc:get_pwm(FIGURE_8_CH)
    
    -- 値が1つでも取れなければ何もしない(すべてのスイッチの接続確認)
    if not fail_pwm or not fig8_pwm then
        return update, 100
    end

    -- if, elseif を使って、優先順位を決めて判定
    -- 【優先度1】耐故障テスト (緑 / Green)
    if fail_pwm > 1800 then
        set_led_color(0, 50, 0)
        
    -- 【優先度2】8の字飛行 (青 / Blue)
    elseif fig8_pwm > 1800 then
        set_led_color(0, 0, 50)
        
    -- 【ハンズオフ飛行中以外 (すべてのミッションがOFFの時)】
    else
        -- 通常状態の赤色 (Red)
        set_led_color(50, 0, 0)
    end

    -- 100ミリ秒(0.1秒)後にまた色をチェックする
    return update, 100
end

gcs:send_text(6, "LED Mode Switch Script Loaded")
return update()

