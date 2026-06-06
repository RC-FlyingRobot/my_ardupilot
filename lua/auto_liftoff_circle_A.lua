-- Pattern A: senkai_module.lua を require して旋回を呼び出す
--
-- 動作シーケンス:
--   CH7 ON + アーム済み
--   → GUIDED で 1.5m 離陸
--   → 3秒ホバー
--   → 半径 1.5m で 1周旋回  (senkai_module 使用)
--   → LAND で着陸
--
-- 配置:
--   このファイル        → APM/scripts/
--   senkai_module.lua   → APM/scripts/modules/  ← require() の検索パス

---@diagnostic disable: cast-local-type
---@diagnostic disable: redundant-parameter

local circle = require("senkai_module")

-- ---- 設定 ----------------------------------------------------------------
local TRIGGER_CH         = 7      -- スイッチチャンネル
local PWM_THRESHOLD      = 1800   -- ON と判定する PWM 閾値
local TAKEOFF_ALT_M      = 1.5    -- 離陸目標高度 [m]
local HOVER_DURATION_MS  = 3000   -- ホバー継続時間 [ms]
local INTERVAL_MS        = 50     -- ループ間隔 [ms]  (= circle.sampling_time_s * 1000)

local GUIDED_MODE    = 4
local STABILIZE_MODE = 0
local LAND_MODE      = 9
-- --------------------------------------------------------------------------

local STATE_IDLE    = 0
local STATE_TAKEOFF = 1
local STATE_HOVER   = 2
local STATE_CIRCLE  = 3
local STATE_LAND    = 4

local state              = STATE_IDLE
local hover_start_ms     = nil
local circle_start_ms    = nil
local circle_duration_ms = nil   -- 1周分のミリ秒 (初回旋回開始時に確定)

gcs:send_text(0, "AutoFlight-A: script loaded")

function update()
    local pwm      = rc:get_pwm(TRIGGER_CH)
    local switch_on = pwm and pwm >= PWM_THRESHOLD
    local is_armed  = arming:is_armed()

    -- STATE_IDLE: CH7 ON + アームで離陸シーケンス開始
    if state == STATE_IDLE then
        if switch_on and is_armed then
            if not ahrs:get_relative_position_NED_origin() then
                gcs:send_text(4, "AutoFlight-A: No position fix, waiting...")
                return update, INTERVAL_MS
            end
            if not vehicle:set_mode(GUIDED_MODE) then
                gcs:send_text(4, "AutoFlight-A: Cannot enter GUIDED")
                return update, INTERVAL_MS
            end
            if not vehicle:start_takeoff(TAKEOFF_ALT_M) then
                gcs:send_text(4, "AutoFlight-A: start_takeoff failed")
                vehicle:set_mode(STABILIZE_MODE)
                return update, INTERVAL_MS
            end
            gcs:send_text(6, string.format("AutoFlight-A: Takeoff to %.1fm", TAKEOFF_ALT_M))
            state = STATE_TAKEOFF
        end

    -- STATE_TAKEOFF: 目標高度の 90% に達したらホバーへ
    elseif state == STATE_TAKEOFF then
        local pos = ahrs:get_relative_position_NED_origin()
        if pos and -pos:z() >= TAKEOFF_ALT_M * 0.9 then
            hover_start_ms = millis()
            gcs:send_text(6, "AutoFlight-A: Hovering 3s")
            state = STATE_HOVER
        end

    -- STATE_HOVER: 3秒待って旋回へ
    elseif state == STATE_HOVER then
        if millis() - hover_start_ms >= HOVER_DURATION_MS then
            local pos = ahrs:get_relative_position_NED_origin()
            if not pos then
                gcs:send_text(4, "AutoFlight-A: No position for circle origin")
                return update, INTERVAL_MS
            end
            circle.set_origin(pos)
            circle.reset()
            circle_start_ms  = millis()
            circle_duration_ms = circle.revolution_s() * 1000
            gcs:send_text(6, string.format(
                "AutoFlight-A: Circle start (r=%.1fm, %.1fs)", circle.rad_xy_m, circle.revolution_s()))
            state = STATE_CIRCLE
        end

    -- STATE_CIRCLE: 1周分の時間が経過したら着陸へ
    elseif state == STATE_CIRCLE then
        if millis() - circle_start_ms >= circle_duration_ms then
            if vehicle:set_mode(LAND_MODE) then
                gcs:send_text(6, "AutoFlight-A: Circle done, landing")
                state = STATE_LAND
            else
                gcs:send_text(4, "AutoFlight-A: Cannot enter LAND mode")
            end
        else
            local tgt_pos, tgt_vel = circle.step()
            if not vehicle:set_target_posvel_NED(tgt_pos, tgt_vel) then
                gcs:send_text(0, "AutoFlight-A: set_target_posvel_NED failed")
            end
        end

    -- STATE_LAND: ディスアームしたらアイドルへリセット
    elseif state == STATE_LAND then
        if not is_armed then
            state = STATE_IDLE
            gcs:send_text(6, "AutoFlight-A: Disarmed, reset to idle")
        end
    end

    return update, INTERVAL_MS
end

return update()
