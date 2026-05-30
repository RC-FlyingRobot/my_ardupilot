-- Commands copter to fly circle trajectory using posvel method in guided mode.
-- The trajectory start from the current location
--
-- CAUTION: This script only works for Copter.
-- This script start when the in GUIDED mode and above 5 meter.
--      1) arm and takeoff to above 5 m
--      2) switch to GUIDED mode
--      3) the vehilce will follow a circle in clockwise direction with increasing speed until ramp_up_time_s time has passed.
--      4) switch out of and into the GUIDED mode any time to restart the trajectory from the start.

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
local copter_guided_mode_num = 4
local theta = 0.0
local time = 0.0
local test_start_location = Vector3f()
local return_mode_num = nil
local circle_active = false

gcs:send_text(0,"Script started")
gcs:send_text(0,"Trajectory period: " .. tostring(2 * math.rad(180) / omega_radps))

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

    cur_freq = omega_radps

    -- calculate circle reference position and velocity
    theta = theta + cur_freq*sampling_time_s

    local th_s = math.sin(theta)
    local th_c = math.cos(theta)

    local pos = Vector3f()
    pos:x(rad_xy_m*th_s)
    pos:y(-rad_xy_m*(th_c-1))
    pos:z(0)

    local vel = Vector3f()
    vel:x(cur_freq*rad_xy_m*th_c)
    vel:y(cur_freq*rad_xy_m*th_s)
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

    -- if arming:is_armed() and vehicle:get_mode() == copter_guided_mode_num and -test_start_location:z()>=1.5 then

        -- calculate current position and velocity for circle trajectory
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

        -- calculate test starting location in NED

        -- local cur_loc = ahrs:get_location()
        -- if cur_loc then
        --      test_start_location = cur_loc.get_vector_from_origin_NEU_cm(cur_loc)
        --      if test_start_location then
        --         test_start_location:x(test_start_location:x() * 0.01)
        --         test_start_location:y(test_start_location:y() * 0.01)
        --         test_start_location:z(-test_start_location:z() * 0.01)
        --      end
        -- end
        -- これは元のコード。ahrs:get_location()はGPS用(?)緯度・経度・高度フレームを持つ位置情報
        -- GPS的な位置やHomeとの距離を扱うには便利だが、今回欲しいのはvehicle:set_target_posvel_NED(target_pos, target_vel)
        -- にわたすためのEKF origin基準のNED座標(メートル)
        -- get_location()からだと上のような変換が必要になる。NED_cmは単位がcmで変換が必要、zはUpからDownに符号変換が必要(NEUはNEDとz座標の符号が逆)


        set_start_location()

        -- reset some variable as soon as we are not in guided mode
        time = 0
        theta = 0
    end

    return update, sampling_time_s * 1000
end

return update()
