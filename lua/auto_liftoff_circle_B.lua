-- Pattern B: senkai.lua の旋回ロジックをインライン展開
--
-- 動作シーケンス:
--   CH7 ON + アーム済み
--   → GUIDED で 1.5m 離陸
--   → 3秒ホバー
--   → 半径 1.5m で 1周旋回  (旋回ロジックをこのファイルに直書き)
--   → LAND で着陸

---@diagnostic disable: cast-local-type
---@diagnostic disable: redundant-parameter

-- ---- 設定 ----------------------------------------------------------------
local TRIGGER_CH         = 7      -- スイッチチャンネル
local PWM_THRESHOLD      = 1800   -- ON と判定する PWM 閾値
local TAKEOFF_ALT_M      = 1.5    -- 離陸目標高度 [m]
local HOVER_DURATION_MS  = 3000   -- ホバー継続時間 [ms]
local INTERVAL_MS        = 50     -- ループ間隔 [ms]

-- 旋回パラメータ (senkai.lua 由来)
local RAD_XY_M         = 1.5    -- 旋回半径 [m]
local TARGET_SPEED_MPS = 1.0    -- 接線速度 [m/s]
local OMEGA_RADPS      = TARGET_SPEED_MPS / RAD_XY_M
local SAMPLING_TIME_S  = INTERVAL_MS / 1000.0
local REVOLUTION_MS    = (2 * math.pi / OMEGA_RADPS) * 1000  -- 1周のミリ秒

local ALT_TOLERANCE_M    = 0.2    -- 高度到達判定の許容誤差 [m]
local RF_ORIENT_DOWN     = 25     -- 下向きレンジファインダーの向き番号

local GUIDED_MODE    = 4
local STABILIZE_MODE = 0
local LAND_MODE      = 9

local SWITCH_DEBOUNCE_CNT = 3   -- 3 * 50ms = 150ms
local ALT_CONFIRM_CNT     = 5   -- 5 * 50ms = 250ms
-- --------------------------------------------------------------------------

local STATE_IDLE    = 0
local STATE_TAKEOFF = 1
local STATE_HOVER   = 2
local STATE_CIRCLE  = 3
local STATE_LAND    = 4

local state           = STATE_IDLE
local hover_start_ms  = nil
local circle_start_ms = nil
local land_fail_count = 0
local switch_off_count  = 0
local alt_confirm_count = 0

-- 旋回変数
local theta          = 0.0
local circle_origin  = Vector3f()

-- 旋回中心を現在位置にセット
local function set_circle_origin()
    local pos = ahrs:get_relative_position_NED_origin()
    if not pos then return false end
    circle_origin:x(pos:x())
    circle_origin:y(pos:y())
    circle_origin:z(pos:z())
    return true
end

-- theta を 1 ステップ進め、絶対 NED 位置を返す
local function circle_step()
    theta = theta + OMEGA_RADPS * SAMPLING_TIME_S

    local th_s = math.sin(theta)
    local th_c = math.cos(theta)

    local pos = Vector3f()
    pos:x(RAD_XY_M * th_s)
    pos:y(-RAD_XY_M * (th_c - 1))
    pos:z(0)   -- 相対高度 0 を保持し続けることで高度を固定

    local abs_pos = Vector3f()
    abs_pos:x(pos:x() + circle_origin:x())
    abs_pos:y(pos:y() + circle_origin:y())
    abs_pos:z(pos:z() + circle_origin:z())   -- = circle_origin:z() で旋回開始時の高度に固定
    return abs_pos
end

gcs:send_text(0, "AutoFlight-B: script loaded")

function update()
    local pwm      = rc:get_pwm(TRIGGER_CH)
    local switch_on = pwm and pwm >= PWM_THRESHOLD
    local is_armed  = arming:is_armed()

    -- スイッチOFF または 解除 → デバウンス後に状態リセット
    if not switch_on or not is_armed then
        switch_off_count = switch_off_count + 1
        if switch_off_count < SWITCH_DEBOUNCE_CNT then
            return update, INTERVAL_MS
        end
        if state ~= STATE_IDLE then
            vehicle:set_mode(STABILIZE_MODE)
            gcs:send_text(6, "AutoFlight-B: STABILIZE restored (switch OFF)")
        end
        state             = STATE_IDLE
        hover_start_ms    = nil
        circle_start_ms   = nil
        theta             = 0.0
        land_fail_count   = 0
        switch_off_count  = 0
        alt_confirm_count = 0
        return update, INTERVAL_MS
    else
        switch_off_count = 0
    end

    -- STATE_IDLE: CH7 ON + アームで離陸シーケンス開始
    if state == STATE_IDLE then
        if switch_on and is_armed then
            if not ahrs:get_relative_position_NED_origin() then
                gcs:send_text(4, "AutoFlight-B: No position fix, waiting...")
                return update, INTERVAL_MS
            end
            if not vehicle:set_mode(GUIDED_MODE) then
                gcs:send_text(4, "AutoFlight-B: Cannot enter GUIDED")
                return update, INTERVAL_MS
            end
            if not vehicle:start_takeoff(TAKEOFF_ALT_M) then
                gcs:send_text(4, "AutoFlight-B: start_takeoff failed")
                vehicle:set_mode(STABILIZE_MODE)
                return update, INTERVAL_MS
            end
            gcs:send_text(6, string.format("AutoFlight-B: Takeoff to %.1fm", TAKEOFF_ALT_M))
            state = STATE_TAKEOFF
        end

    -- STATE_TAKEOFF: 目標高度に達したらホバーへ
    elseif state == STATE_TAKEOFF then
        if rangefinder:has_data_orient(RF_ORIENT_DOWN) then
            local current_alt = rangefinder:distance_orient(RF_ORIENT_DOWN)
            if math.abs(current_alt - TAKEOFF_ALT_M) < ALT_TOLERANCE_M then
                alt_confirm_count = alt_confirm_count + 1
                if alt_confirm_count >= ALT_CONFIRM_CNT then
                    alt_confirm_count = 0
                    hover_start_ms = millis()
                    gcs:send_text(6, string.format("AutoFlight-B: Reached %.2fm, hovering 3s", current_alt))
                    state = STATE_HOVER
                end
            else
                alt_confirm_count = 0
            end
        else
            -- AHRS フォールバック: 90% 到達で遷移
            local pos = ahrs:get_relative_position_NED_origin()
            if pos and -pos:z() >= TAKEOFF_ALT_M * 0.9 then
                alt_confirm_count = alt_confirm_count + 1
                if alt_confirm_count >= ALT_CONFIRM_CNT then
                    alt_confirm_count = 0
                    hover_start_ms = millis()
                    gcs:send_text(6, "AutoFlight-B: Hovering 3s (AHRS fallback)")
                    state = STATE_HOVER
                end
            else
                alt_confirm_count = 0
            end
        end

    -- STATE_HOVER: 3秒待って旋回へ
    elseif state == STATE_HOVER then
        if millis() - hover_start_ms >= HOVER_DURATION_MS then
            if not set_circle_origin() then
                gcs:send_text(4, "AutoFlight-B: No position for circle origin")
                return update, INTERVAL_MS
            end
            theta            = 0.0
            circle_start_ms  = millis()
            gcs:send_text(6, string.format(
                "AutoFlight-B: Circle start (r=%.1fm, %.1fs)", RAD_XY_M, REVOLUTION_MS / 1000))
            state = STATE_CIRCLE
        end

    -- STATE_CIRCLE: 1周分の時間が経過したら着陸へ
    elseif state == STATE_CIRCLE then
        if millis() - circle_start_ms >= REVOLUTION_MS then
            if vehicle:set_mode(LAND_MODE) then
                gcs:send_text(6, "AutoFlight-B: Circle done, landing")
                land_fail_count = 0
                state = STATE_LAND
            else
                land_fail_count = land_fail_count + 1
                gcs:send_text(4, string.format("AutoFlight-B: Cannot enter LAND mode (%d)", land_fail_count))
                if land_fail_count >= 5 then
                    vehicle:set_mode(STABILIZE_MODE)
                    gcs:send_text(4, "AutoFlight-B: LAND failed 5x, STABILIZE fallback")
                    land_fail_count = 0
                    state = STATE_IDLE
                end
            end
        else
            local tgt_pos = circle_step()
            if not vehicle:set_target_pos_NED(tgt_pos, false, 0, false, 0, false, false) then
                gcs:send_text(0, "AutoFlight-B: set_target_pos_NED failed")
            end
        end

    -- STATE_LAND: ディスアームまたはスイッチOFFをガードに委任して待機
    elseif state == STATE_LAND then
        if not is_armed then
            state = STATE_IDLE
            gcs:send_text(6, "AutoFlight-B: Disarmed, reset to idle")
        end
    end

    return update, INTERVAL_MS
end

return update()


/*
2026/06/14 13:51:14 : AutoFlight-B: STABILIZE restored (switch OFF)
2026/06/14 13:51:09 : AutoFlight-B: Circle done, landing
2026/06/14 13:51:00 : AutoFlight-B: Circle start (r=1.5m, 9.4s)
2026/06/14 13:50:57 : AutoFlight-B: Reached 1.41m, hovering 3s
2026/06/14 13:50:55 : AutoFlight-B: Takeoff to 1.5m
2026/06/14 13:50:55 : AutoFlight-B: Cannot enter GUIDED
2026/06/14 13:50:55 : Mode change to Guided failed: requires position
...
2026/06/14 13:50:48 : AutoFlight-B: Cannot enter GUIDED
2026/06/14 13:50:48 : Mode change to Guided failed: requires position
2026/06/14 13:49:44 : EKF3 IMU0 fusing optical flow
2026/06/14 13:49:44 : EKF3 IMU0 started relative aiding
*/