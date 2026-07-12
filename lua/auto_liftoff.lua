-- CH7 ON  → GUIDEDモードで高度1mにホバリング (地上からの離陸)
-- CH7 OFF → STABILIZEモードに復帰して手動操縦可能に
-- ホバリング開始から5秒後に自動着陸

local AUTO_FLIGHT_CH    = 7      -- トリガーチャンネル番号
local PWM_THRESHOLD     = 1800   -- スイッチONと判定するPWM値
local HOVER_ALT_M       = 1.0    -- ホバリング高度 (m)
local HOVER_DURATION_MS = 5000   -- ホバリング継続時間: 5秒
local INTERVAL_MS       = 100    -- ループ間隔 (ms)
local ALT_TOLERANCE_M   = 0.2    -- 高度到達判定の許容誤差 (m)
local RF_ORIENT_DOWN    = 25     -- 下向きレンジファインダーの向き番号

local copter_guided_mode    = 4  -- GUIDEDモード番号
local copter_stabilize_mode = 0  -- STABILIZEモード番号
local copter_land_mode      = 9  -- LANDモード番号

-- stage: 0=待機, 1=上昇確認中, 2=ホバリング中
local stage          = 0
local hover_start_ms = nil

function update()
    local pwm       = rc:get_pwm(AUTO_FLIGHT_CH)
    local switch_on = pwm and pwm >= PWM_THRESHOLD
    local is_armed  = arming:is_armed()

    -- スイッチOFF または 解除 → 状態リセット
    if not switch_on or not is_armed then
        if stage > 0 then
            stage          = 0
            hover_start_ms = nil
            vehicle:set_mode(copter_stabilize_mode)
            gcs:send_text(6, "Auto Hover: STABILIZE restored (CH7 OFF)")
        end
        return update, INTERVAL_MS
    end

    -- stage 0: 離陸コマンド送出
    if stage == 0 then
        if not ahrs:get_relative_position_NED_origin() then
            gcs:send_text(4, "Auto Hover: No position fix, waiting...")
            return update, INTERVAL_MS
        end

        if not vehicle:set_mode(copter_guided_mode) then
            gcs:send_text(4, "Auto Hover: Failed to enter GUIDED mode")
            return update, INTERVAL_MS
        end

        vehicle:start_takeoff(HOVER_ALT_M)
        stage = 1
        gcs:send_text(6, "Auto Hover: Climbing to 1.5m")

    -- stage 1: レンジファインダーで高度到達を確認してから位置ロック
    elseif stage == 1 then
        if rangefinder:has_data_orient(RF_ORIENT_DOWN) then
            local current_alt = rangefinder:distance_orient(RF_ORIENT_DOWN)
            local alt_err = math.abs(current_alt - HOVER_ALT_M)
            gcs:send_text(6, string.format("Auto Hover: alt err %.2fm", alt_err))

            if alt_err < ALT_TOLERANCE_M then
                hover_start_ms = millis()
                stage = 2
                gcs:send_text(6, string.format("Auto Hover: Reached %.2fm, hovering 5s", current_alt))
            end
        else
            gcs:send_text(6, "Auto Hover: Rangefinder NO DATA")
        end

    -- stage 2: ホバリング中 → 5秒後に自動着陸
    elseif stage == 2 then
        if millis() - hover_start_ms >= HOVER_DURATION_MS then
            hover_start_ms = nil
            if not vehicle:set_mode(copter_land_mode) then
                gcs:send_text(4, "Auto Hover: Failed to enter LAND mode")
            else
                gcs:send_text(6, "Auto Hover: Auto landing after 5s")
            end
        end
    end

    return update, INTERVAL_MS
end

return update()
