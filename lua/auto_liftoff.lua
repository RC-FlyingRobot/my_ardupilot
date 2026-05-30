-- auto_liftoff.lua
-- State-machine flight script for ArduPilot Copter.
--   CH7 ON  -> GUIDED takeoff -> hover 3 s -> figure-eight circles -> auto land
--   CH7 OFF -> abort to STABILIZE at any time (except IDLE)

local senkai = require('senkai')

-- Constants
local AUTO_FLIGHT_CH    = 7
local PWM_THRESHOLD     = 1800
local HOVER_ALT_M       = 1.0
local HOVER_ALT_CM      = 100
local LIFTOFF_CONFIRM_M = 0.8
local HOVER_DURATION_MS = 3000
local NUM_LAPS          = 2
local INTERVAL_MS       = 100

local MODE_STABILIZE = 0
local MODE_GUIDED    = 4
local MODE_LAND      = 9

-- States
local STATE_IDLE    = 0
local STATE_LIFTOFF = 1
local STATE_HOVER   = 2
local STATE_SENKAI  = 3
local STATE_LANDING = 4

-- State variables
local state          = STATE_IDLE
local hover_start_ms = nil

-- Abort helper: reset everything and switch to STABILIZE
local function abort_to_stabilize()
    state          = STATE_IDLE
    hover_start_ms = nil
    senkai.reset()
    vehicle:set_mode(MODE_STABILIZE)
    gcs:send_text(6, "Auto Flight: Aborted to STABILIZE (CH7 OFF)")
end

function update()
    local pwm = rc:get_pwm(AUTO_FLIGHT_CH)
    if not pwm then return update, INTERVAL_MS end

    local switch_on = pwm > PWM_THRESHOLD

    ---------- CH7 OFF guard (before any state logic) ----------
    if state ~= STATE_IDLE and not switch_on then
        abort_to_stabilize()
        return update, INTERVAL_MS
    end

    ---------- STATE_IDLE ----------
    if state == STATE_IDLE then
        if switch_on and arming:is_armed() then
            -- Need position fix
            if not ahrs:get_relative_position_NED_origin() then
                gcs:send_text(4, "Auto Flight: No position fix, waiting...")
                return update, INTERVAL_MS
            end

            if not vehicle:set_mode(MODE_GUIDED) then
                gcs:send_text(4, "Auto Flight: Failed to enter GUIDED mode")
                return update, INTERVAL_MS
            end

            -- Attempt ground takeoff; fall back to set_target_location if airborne
            if not vehicle:start_takeoff(HOVER_ALT_M) then
                local curr_loc = ahrs:get_location()
                if not curr_loc then
                    gcs:send_text(4, "Auto Flight: Failed to get location")
                    vehicle:set_mode(MODE_STABILIZE)
                    return update, INTERVAL_MS
                end

                curr_loc.alt          = HOVER_ALT_CM
                curr_loc.relative_alt = true

                if not vehicle:set_target_location(curr_loc) then
                    gcs:send_text(4, "Auto Flight: Failed to set target location")
                    vehicle:set_mode(MODE_STABILIZE)
                    return update, INTERVAL_MS
                end
            end

            state = STATE_LIFTOFF
            gcs:send_text(6, "Auto Flight: Liftoff commanded (CH7 ON)")
        end

    ---------- STATE_LIFTOFF ----------
    elseif state == STATE_LIFTOFF then
        local ned = ahrs:get_relative_position_NED_origin()
        if ned and (-ned:z()) >= LIFTOFF_CONFIRM_M then
            hover_start_ms = millis()
            state = STATE_HOVER
            gcs:send_text(6, "Auto Flight: Altitude reached, hovering 3 s")
        end

    ---------- STATE_HOVER ----------
    elseif state == STATE_HOVER then
        if hover_start_ms and (millis() - hover_start_ms >= HOVER_DURATION_MS) then
            if not senkai.set_start_location() then
                gcs:send_text(4, "Auto Flight: Position unavailable, retrying...")
                return update, INTERVAL_MS
            end
            senkai.reset()
            state = STATE_SENKAI
            hover_start_ms = nil
            gcs:send_text(6, "Auto Flight: Starting circle pattern")
            -- Fall through to first circle tick below
        end

    ---------- STATE_SENKAI ----------
    elseif state == STATE_SENKAI then
        -- Check lap completion BEFORE computing the next step
        if senkai.get_theta() >= NUM_LAPS * 2 * math.pi then
            if not vehicle:set_mode(MODE_LAND) then
                gcs:send_text(4, "Auto Flight: Failed to enter LAND mode")
            else
                gcs:send_text(6, "Auto Flight: Laps complete, landing")
            end
            state = STATE_LANDING
            return update, INTERVAL_MS
        end

        local pos, vel = senkai.circle()
        if not vehicle:set_target_posvel_NED(pos + senkai.get_start_loc(), vel) then
            gcs:send_text(4, "Auto Flight: Failed to send posvel")
        end
        return update, senkai.sampling_time_s * 1000

    ---------- STATE_LANDING ----------
    elseif state == STATE_LANDING then
        if not arming:is_armed() then
            state = STATE_IDLE
            gcs:send_text(6, "Auto Flight: Landed and disarmed, resetting")
        end
    end

    return update, INTERVAL_MS
end

gcs:send_text(6, "Auto Flight Script Loaded (CH7)")
return update()
