local SERVO_CH = 1   -- 使うサーボ出力（SERVO1）
local MIN_PWM = 1300
local MAX_PWM = 1700
local STOP_PWM = 1500

-- 0〜3.3Vを14段階に変換
function voltage_to_level(v)
    local level = math.floor((v / 3.3) * 14) + 1

    if level < 1 then level = 1 end
    if level > 14 then level = 14 end

    return level
end

-- レベル → PWM出力
function level_to_pwm(level)
    if level == 7 then
        return STOP_PWM
    end

    local step = 50  -- 段階ごとの変化量（調整ポイント）

    if level < 7 then
        -- マイナス回転
        return STOP_PWM - (7 - level) * step
    else
        -- プラス回転
        return STOP_PWM + (level - 7) * step
    end
end

function update()
    local curr = battery:current_amps(0)

    if curr ~= nil then
        local level = voltage_to_level(curr)
        local pwm = level_to_pwm(level)

        -- サーボ出力
        SRV_Channels:set_output_pwm(SERVO_CH, pwm)

        -- デバッグ表示
        gcs:send_text(0, string.format("V=%.2f L=%d PWM=%d", curr, level, pwm))
    end

    return update, 100
end

return update()

/*current（=0〜約3.3V） を読む
それを 14段階に分割
7を中心（停止）
1〜6 → 逆回転（負）
8〜14 → 正回転（正）
段階ごとに速度を変える**/