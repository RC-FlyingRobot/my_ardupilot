-- senkai_ch9.lua
-- CH9 ON  -> circle trajectory in GUIDED mode (manual trigger)
-- CH9 OFF -> restore previous mode
-- Delegates all circle math to the senkai module.

local senkai = require('senkai')

local CH9_THRESHOLD          = 1500
local GUIDED_MODE            = 4
local SAMPLING_INTERVAL_MS   = senkai.sampling_time_s * 1000

local circle_active   = false
local return_mode_num = nil

local function restore_mode(reason)
    if return_mode_num == nil then return end
    if vehicle:set_mode(return_mode_num) then
        gcs:send_text(6, reason .. string.format(" (mode %d)", return_mode_num))
    else
        gcs:send_text(4, "Senkai CH9: Failed to restore previous mode")
    end
    return_mode_num = nil
end

function update()
    local ch9_pwm = rc:get_pwm(9)
    if not ch9_pwm then return update, 1000 end

    if arming:is_armed() and ch9_pwm > CH9_THRESHOLD then
        if not circle_active then
            return_mode_num = vehicle:get_mode()
            if not senkai.set_start_location() then
                gcs:send_text(4, "Senkai CH9: Position unavailable from EKF origin")
                return update, 500
            end
            if not vehicle:set_mode(GUIDED_MODE) then
                gcs:send_text(4, "Senkai CH9: Failed to change to GUIDED")
                return update, 500
            end
            senkai.reset()
            circle_active = true
            gcs:send_text(6, "Senkai CH9: Circle started (GUIDED)")
        end

        local pos, vel = senkai.circle()
        if not vehicle:set_target_posvel_NED(pos + senkai.get_start_loc(), vel) then
            gcs:send_text(0, "Senkai CH9: posvel send failed at theta=" .. tostring(senkai.get_theta()))
        end
        return update, SAMPLING_INTERVAL_MS
    else
        if circle_active then
            restore_mode("Senkai CH9: Restored previous mode")
            senkai.reset()
            circle_active = false
        end
        senkai.set_start_location()
        return update, SAMPLING_INTERVAL_MS
    end
end

gcs:send_text(6, "Senkai CH9 Script Loaded")
return update()
