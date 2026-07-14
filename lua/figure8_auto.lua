-- Commands Copter to fly a figure-eight made from two tangent circles.
-- The trajectory starts from the current location and uses NED position and velocity targets.
--
-- CAUTION: This script only works for Copter.
--      1) Arm and take off to a safe altitude.
--      2) Raise RC channel 6 above ch6_threshold.
--      3) The vehicle enters GUIDED and flies two circles in opposite directions.
--      4) Lower RC channel 6 to stop and restore the previous mode.

---@diagnostic disable: cast-local-type
---@diagnostic disable: redundant-parameter

-- Edit these variables
local rad_xy_m = 1.5
local target_speed_xy_mps = 1.0
local sampling_time_s = 0.05
local ch6_threshold = 1500

-- Fixed variables
local omega_radps = target_speed_xy_mps / rad_xy_m
local copter_guided_mode_num = 4
local full_circle_rad = 2.0 * math.pi
local theta = 0.0
local circle_direction = 1.0
local test_start_location = Vector3f()
local return_mode_num = nil
local figure8_active = false
local FIGURE_8_CH = 6

gcs:send_text(0, "Figure8: Script started")
gcs:send_text(0, "Figure8 period: " .. tostring(2.0 * full_circle_rad / omega_radps))

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

local function figure8_target()
    theta = theta + omega_radps * sampling_time_s
    if theta >= full_circle_rad then
        theta = theta - full_circle_rad
        circle_direction = -circle_direction
    end

    local theta_sin = math.sin(theta)
    local theta_cos = math.cos(theta)

    local pos = Vector3f()
    pos:x(rad_xy_m * theta_sin)
    pos:y(circle_direction * rad_xy_m * (1.0 - theta_cos))
    pos:z(0)

    local vel = Vector3f()
    vel:x(omega_radps * rad_xy_m * theta_cos)
    vel:y(circle_direction * omega_radps * rad_xy_m * theta_sin)
    vel:z(0)

    return pos, vel
end

local function update()
    local ch6_pwm = rc:get_pwm(FIGURE_8_CH)
    if not ch6_pwm then
        return update, 1000
    end

    if arming:is_armed() and ch6_pwm > ch6_threshold then
        if not figure8_active then
            return_mode_num = vehicle:get_mode()
            if not set_start_location() then
                gcs:send_text(4, "Figure8: Position unavailable from EKF origin")
                return update, 500
            end
            if vehicle:set_mode(copter_guided_mode_num) then
                figure8_active = true
                gcs:send_text(6, "Figure8: Changed to GUIDED")
            else
                gcs:send_text(4, "Figure8: Failed to change to GUIDED")
                return update, 500
            end
        end

        local target_pos, target_vel = figure8_target()

        if not vehicle:set_target_posvel_NED(target_pos + test_start_location, target_vel) then
            gcs:send_text(4, "Figure8: Failed to send target")
        end
    else
        if figure8_active then
            restore_return_mode("Figure8: Restored previous mode")
            figure8_active = false
        end

        set_start_location()
        theta = 0
        circle_direction = 1.0
    end

    return update, sampling_time_s * 1000
end

return update()
