-- Commands copter to fly double-circle figure-eight trajectory using posvel method in guided mode.
-- The trajectory starts from the current location.
--
-- CAUTION: This script only works for Copter.

---@diagnostic disable: cast-local-type
---@diagnostic disable: redundant-parameter

-- Edit these variables
local rad_xy_m = 1.5   -- circle radius in xy plane in m
local target_speed_xy_mps = 1.0     -- maximum target speed in m/s
local ramp_up_time_s = 3.0     -- time to reach target_speed_xy_mps in second
local sampling_time_s = 0.05    -- sampling time of script
local ch9_threshold = 1500

-- Fixed variables
local omega_radps = target_speed_xy_mps/rad_xy_m
local two_pi = 2 * math.pi
local copter_guided_mode_num = 4
local theta = 0.0
local time = 0.0
local test_start_location = Vector3f()
local return_mode_num = nil
local circle_active = false

gcs:send_text(0,"Script started")
gcs:send_text(0,"Double circle period: " .. tostring(2 * two_pi / omega_radps))

local function restore_return_mode(reason)
    if return_mode_num == nil then
        return
    end

    if vehicle:set_mode(return_mode_num) then
        gcs:send_text(6, reason .. string.format(" (mode %d)", return_mode_num))
    else
        gcs:send_text(4, "Failed to restore previous mode")
    end
    return_mode_num = nil
end

local function set_start_location()
    local cur_pos_ned = ahrs:get_relative_position_NED_origin()
    if cur_pos_ned == nil then
        return false
    end

    test_start_location:x(cur_pos_ned:x())
    test_start_location:y(cur_pos_ned:y())
    test_start_location:z(cur_pos_ned:z())
    return true
end

function circle()
    local cur_freq
    -- increase target speed lineary with time until ramp_up_time_s is reached
    if time <= ramp_up_time_s then
        cur_freq = omega_radps*(time/ramp_up_time_s)^2
    else
        cur_freq = omega_radps
    end

    -- calculate double-circle reference position and velocity
    theta = theta + cur_freq*sampling_time_s

    local circle_num = math.floor(theta / two_pi)
    local circle_theta = theta - circle_num * two_pi
    local circle_side = 1.0
    if circle_num % 2 == 1 then
        circle_side = -1.0
    end

    local th_s = math.sin(circle_theta)
    local th_c = math.cos(circle_theta)

    local pos = Vector3f()
    pos:x(rad_xy_m*th_s)
    pos:y(circle_side*rad_xy_m*(1-th_c))
    pos:z(0)

    local vel = Vector3f()
    vel:x(cur_freq*rad_xy_m*th_c)
    vel:y(circle_side*cur_freq*rad_xy_m*th_s)
    vel:z(0)

    return pos, vel
end

function update()

    local ch9_pwm = rc:get_pwm(9)
    if not ch9_pwm then
        return update, 1000
    end

    if arming:is_armed() and ch9_pwm > ch9_threshold then
        if not circle_active then
            return_mode_num = vehicle:get_mode()
            if not set_start_location() then
                gcs:send_text(4, "Circle: Position unavailable from EKF origin")
                return update, 500
            end
            if vehicle:set_mode(copter_guided_mode_num) then
                circle_active = true
                gcs:send_text(6, "Circle: Changed to GUIDED")
            else
                gcs:send_text(4, "Circle: Failed to change to GUIDED")
                return update, 500
            end
        end

        -- calculate current position and velocity for double-circle trajectory
        local target_pos, target_vel = circle()

        -- advance the time
        time = time + sampling_time_s

        -- send posvel request
        if not vehicle:set_target_posvel_NED(target_pos+test_start_location, target_vel) then
            gcs:send_text(0, "Failed to send target posvel at " .. tostring(time) .. " seconds")
        end
    else
        if circle_active then
            restore_return_mode("Circle: Restored previous mode")
            circle_active = false
        end

        set_start_location()

        -- reset some variable as soon as we are not in guided mode
        time = 0
        theta = 0
    end

    return update, sampling_time_s * 1000
end

return update()
