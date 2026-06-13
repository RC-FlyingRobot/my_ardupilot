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

local GUIDED_MODE    = 4
local STABILIZE_MODE = 0
local LAND_MODE      = 9
-- --------------------------------------------------------------------------

local STATE_IDLE    = 0
local STATE_TAKEOFF = 1
local STATE_HOVER   = 2
local STATE_CIRCLE  = 3
local STATE_LAND    = 4

local state           = STATE_IDLE
local hover_start_ms  = nil
local circle_start_ms = nil

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

-- theta を 1 ステップ進め、絶対 NED 位置と速度を返す (senkai.lua の circle() と同等)
local function circle_step()
    theta = theta + OMEGA_RADPS * SAMPLING_TIME_S

    local th_s = math.sin(theta)
    local th_c = math.cos(theta)

    local rel_pos = Vector3f()
    rel_pos:x(RAD_XY_M * th_s)
    rel_pos:y(-RAD_XY_M * (th_c - 1))
    rel_pos:z(0)

    local vel = Vector3f()
    vel:x(OMEGA_RADPS * RAD_XY_M * th_c)
    vel:y(OMEGA_RADPS * RAD_XY_M * th_s)
    vel:z(0)

    return rel_pos + circle_origin, vel
end

gcs:send_text(0, "AutoFlight-B: script loaded")

function update()
    local pwm      = rc:get_pwm(TRIGGER_CH)
    local switch_on = pwm and pwm >= PWM_THRESHOLD
    local is_armed  = arming:is_armed()

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

    -- STATE_TAKEOFF: 目標高度の 90% に達したらホバーへ
    elseif state == STATE_TAKEOFF then
        local pos = ahrs:get_relative_position_NED_origin()
        if pos and -pos:z() >= TAKEOFF_ALT_M * 0.9 then
            hover_start_ms = millis()
            gcs:send_text(6, "AutoFlight-B: Hovering 3s")
            state = STATE_HOVER
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
                state = STATE_LAND
            else
                gcs:send_text(4, "AutoFlight-B: Cannot enter LAND mode")
            end
        else
            local tgt_pos, tgt_vel = circle_step()
            if not vehicle:set_target_posvel_NED(tgt_pos, tgt_vel) then
                gcs:send_text(0, "AutoFlight-B: set_target_posvel_NED failed")
            end
        end

    -- STATE_LAND: ディスアームしたらアイドルへリセット
    elseif state == STATE_LAND then
        if not is_armed then
            state = STATE_IDLE
            gcs:send_text(6, "AutoFlight-B: Disarmed, reset to idle")
        end
    end

    return update, INTERVAL_MS
end

return update()
