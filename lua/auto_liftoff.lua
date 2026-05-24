-- auto_liftoff.lua
-- CH7 ON  → GUIDEDモードで高度1mにホバリング (地上からの離陸 / 飛行中の高度変更 両対応)
-- CH7 OFF → LOITERモードに復帰して操縦可能に

local AUTO_FLIGHT_CH = 7      -- トリガーチャンネル番号
local PWM_THRESHOLD  = 1800   -- スイッチONと判定するPWM値
local HOVER_ALT_CM   = 100    -- ホバリング高度: 100 cm = 1 m (ホームから相対)
local INTERVAL_MS    = 100    -- ループ間隔 (ms)

local copter_guided_mode = 4  -- GUIDEDモード番号
local copter_loiter_mode = 5  -- LOITERモード番号

local is_active = false

function update()
    local pwm = rc:get_pwm(AUTO_FLIGHT_CH)
    if not pwm then return update, INTERVAL_MS end

    local switch_on = pwm > PWM_THRESHOLD
    local is_armed  = arming:is_armed()

    -- スイッチON & アーム済み & 未実行 → ホバリング開始
    if switch_on and is_armed and not is_active then
        if not vehicle:set_mode(copter_guided_mode) then
            gcs:send_text(4, "Auto Hover: Failed to enter GUIDED mode")
            return update, INTERVAL_MS
        end

        is_active = true

        -- まず地上からの離陸を試みる (飛行中の場合はfalseが返る)
        if not vehicle:start_takeoff(1.0) then
            -- 飛行中の場合: 現在の緯度・経度を維持しつつ高度だけ1mに変更
            local curr_loc = ahrs:get_location()
            if not curr_loc then
                gcs:send_text(4, "Auto Hover: Failed to get location")
                is_active = false
                vehicle:set_mode(copter_loiter_mode)
                return update, INTERVAL_MS
            end

            curr_loc.alt          = HOVER_ALT_CM
            curr_loc.relative_alt = true

            if not vehicle:set_target_location(curr_loc) then
                gcs:send_text(4, "Auto Hover: Failed to set target location")
                is_active = false
                vehicle:set_mode(copter_loiter_mode)
                return update, INTERVAL_MS
            end
        end

        gcs:send_text(6, "Auto Hover: Moving to 1m hover (CH7 ON)")

    -- スイッチOFF & 実行中 → LOITERに復帰して手動操縦へ
    elseif not switch_on and is_active then
        is_active = false
        vehicle:set_mode(copter_loiter_mode)
        gcs:send_text(6, "Auto Hover: LOITER restored (CH7 OFF)")
    end

    return update, INTERVAL_MS
end

gcs:send_text(6, "Auto Hover Script Loaded (CH7)")
return update()
