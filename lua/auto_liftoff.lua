-- auto_liftoff.lua
-- CH7 ON  → GUIDEDモードで高度1mにホバリング (地上からの離陸 / 飛行中の高度変更 両対応)
-- CH7 OFF → STABILIZEモードに復帰して手動操縦可能に
-- ホバリング開始から5秒後に自動着陸

local AUTO_FLIGHT_CH    = 7      -- トリガーチャンネル番号
local PWM_THRESHOLD     = 1800   -- スイッチONと判定するPWM値
local HOVER_ALT_CM      = 100    -- ホバリング高度: 100 cm = 1 m (ホームから相対)
local HOVER_DURATION_MS = 5000   -- ホバリング継続時間: 5秒
local INTERVAL_MS       = 100    -- ループ間隔 (ms)

local copter_guided_mode    = 4  -- GUIDEDモード番号
local copter_stabilize_mode = 0  -- STABILIZEモード番号
local copter_land_mode      = 9  -- LANDモード番号

local is_active      = false
local hover_start_ms = nil

function update()
    local pwm = rc:get_pwm(AUTO_FLIGHT_CH)
    if not pwm then return update, INTERVAL_MS end

    local switch_on = pwm > PWM_THRESHOLD
    local is_armed  = arming:is_armed()

    -- スイッチON & アーム済み & 未実行 → ホバリング開始
    if switch_on and is_armed and not is_active then
        -- GUIDEDモードには位置推定が必要なため、EKF原点からの相対位置で確認する
        if not ahrs:get_relative_position_NED_origin() then
            gcs:send_text(4, "Auto Hover: No position fix, waiting...")
            return update, INTERVAL_MS
        end

        if not vehicle:set_mode(copter_guided_mode) then
            gcs:send_text(4, "Auto Hover: Failed to enter GUIDED mode")
            return update, INTERVAL_MS
        end

        is_active      = true
        hover_start_ms = millis()

        -- まず地上からの離陸を試みる (飛行中の場合はfalseが返る)
        if not vehicle:start_takeoff(1.0) then
            -- 飛行中の場合: 現在の緯度・経度を維持しつつ高度だけ1mに変更
            local curr_loc = ahrs:get_location()
            if not curr_loc then
                gcs:send_text(4, "Auto Hover: Failed to get location")
                is_active      = false
                hover_start_ms = nil
                vehicle:set_mode(copter_stabilize_mode)
                return update, INTERVAL_MS
            end

            curr_loc.alt          = HOVER_ALT_CM
            curr_loc.relative_alt = true

            if not vehicle:set_target_location(curr_loc) then
                gcs:send_text(4, "Auto Hover: Failed to set target location")
                is_active      = false
                hover_start_ms = nil
                vehicle:set_mode(copter_stabilize_mode)
                return update, INTERVAL_MS
            end
        end

        gcs:send_text(6, "Auto Hover: Moving to 1m hover (CH7 ON)")

    -- ホバリング中 → タイマー満了またはスイッチOFFで終了
    elseif is_active then
        if hover_start_ms and (millis() - hover_start_ms >= HOVER_DURATION_MS) then
            -- 5秒経過で自動着陸
            -- is_active は true のまま: CH7 が ON の間は再ホバーを防ぐ
            hover_start_ms = nil
            if not vehicle:set_mode(copter_land_mode) then
                gcs:send_text(4, "Auto Hover: Failed to enter LAND mode")
            else
                gcs:send_text(6, "Auto Hover: Auto landing after 5s")
            end
        elseif not switch_on then
            -- スイッチOFF → STABILIZEに復帰して手動操縦へ
            is_active      = false
            hover_start_ms = nil
            vehicle:set_mode(copter_stabilize_mode)
            gcs:send_text(6, "Auto Hover: STABILIZE restored (CH7 OFF)")
        end
    end

    return update, INTERVAL_MS
end

gcs:send_text(6, "Auto Hover Script Loaded (CH7)")
return update()
