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
local lidar_filter_alpha = 0.2
local lidar_max_z_correction_m = 0.3
local lidar_max_z_rate_mps = 0.3
local lidar_timeout_ms = 1000
local altitude_report_interval_ms = 1000

-- Fixed variables
local omega_radps = target_speed_xy_mps/rad_xy_m
local copter_guided_mode_num = 4
local rotation_downward = 25 -- ROTATION_PITCH_270
local rangefinder_status_good = 4
local theta = 0.0
local time = 0.0
local test_start_location = Vector3f()
local return_mode_num = nil
local circle_active = false
local lidar_fault_latched = false
local lidar_target_height_m = nil
local lidar_filtered_height_m = nil
local lidar_last_height_m = nil
local lidar_last_valid_ms = 0
local lidar_status = 0
local lidar_quality = -1
local z_correction_m = 0.0
local z_correction_rate_mps = 0.0
local last_altitude_report_ms = 0

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

local function constrain(value, minimum, maximum)
    return math.max(minimum, math.min(maximum, value))
end

local function get_lidar_height_m()
    lidar_status = rangefinder:status_orient(rotation_downward)
    lidar_quality = rangefinder:signal_quality_pct_orient(rotation_downward)
    if lidar_status ~= rangefinder_status_good or
       not rangefinder:has_data_orient(rotation_downward) then
        return nil
    end

    local distance_m = rangefinder:distance_orient(rotation_downward)
    local roll_rad = ahrs:get_roll_rad()
    local pitch_rad = ahrs:get_pitch_rad()
    if distance_m <= 0 or roll_rad == nil or pitch_rad == nil then
        return nil
    end

    -- Convert the slant range along the sensor axis to vertical height.
    return distance_m * math.cos(roll_rad) * math.cos(pitch_rad)
end

local function set_start_location()
    local cur_pos_ned = ahrs:get_relative_position_NED_origin()
    if cur_pos_ned == nil then
        return false
    end

    local lidar_height_m = get_lidar_height_m()
    if lidar_height_m == nil then
        return false
    end

    test_start_location:x(cur_pos_ned:x())
    test_start_location:y(cur_pos_ned:y())
    test_start_location:z(cur_pos_ned:z())
    lidar_target_height_m = lidar_height_m
    lidar_filtered_height_m = lidar_height_m
    lidar_last_height_m = lidar_height_m
    lidar_last_valid_ms = millis()
    z_correction_m = 0.0
    z_correction_rate_mps = 0.0
    return true
end

local function update_lidar_height_control()
    local lidar_height_m = get_lidar_height_m()
    if lidar_height_m == nil then
        z_correction_rate_mps = 0.0
        return millis() - lidar_last_valid_ms <= lidar_timeout_ms
    end

    lidar_last_valid_ms = millis()
    lidar_last_height_m = lidar_height_m
    lidar_filtered_height_m = lidar_filtered_height_m +
        lidar_filter_alpha * (lidar_height_m - lidar_filtered_height_m)

    -- In NED, positive Z is down. If the LiDAR height is too large,
    -- increase the Z target to command a descent.
    local desired_correction_m = constrain(
        lidar_filtered_height_m - lidar_target_height_m,
        -lidar_max_z_correction_m,
        lidar_max_z_correction_m)
    local max_step_m = lidar_max_z_rate_mps * sampling_time_s
    local correction_step_m = constrain(
        desired_correction_m - z_correction_m,
        -max_step_m,
        max_step_m)

    z_correction_m = z_correction_m + correction_step_m
    z_correction_rate_mps = correction_step_m / sampling_time_s
    return true
end

-- 高度診断ログ:
--   SNHTはスクリプトの更新周期でDataFlashログへ記録する。
--   同じ内容の要約を1秒ごとにGCSへ送信する。
--   Rng  : 機体の傾きを補正した下向きLiDARの対地高度 [m]
--   Ref  : 旋回開始時に記録したLiDAR目標高度 [m]
--   Ekf  : EKF原点基準のNED Zから求めた機体の推定高度 [m]
--   Tgt  : ArduPilotへ指示しているNED Z目標に対応する高度 [m]
--   Corr : 開始時のNED Zへ加えるLiDAR高度補正量 [m]
--   Vz   : NED鉛直補正速度。正の値は下降方向 [m/s]
--   Qual : RangeFinderの信号品質 [%]。-1は取得不可
--   Stat : RangeFinderの状態。4はGood
local function log_altitude(target_z_ned_m)
    local cur_pos_ned = ahrs:get_relative_position_NED_origin()
    if cur_pos_ned == nil or lidar_last_height_m == nil or lidar_target_height_m == nil then
        return
    end

    local ekf_height_m = -cur_pos_ned:z()
    local target_height_m = -target_z_ned_m
    logger:write(
        "SNHT",
        "Rng,Ref,Ekf,Tgt,Corr,Vz,Qual,Stat",
        "ffffffbB",
        lidar_last_height_m,
        lidar_target_height_m,
        ekf_height_m,
        target_height_m,
        z_correction_m,
        z_correction_rate_mps,
        lidar_quality,
        lidar_status)

    local now_ms = millis()
    if last_altitude_report_ms == 0 or
       now_ms - last_altitude_report_ms >= altitude_report_interval_ms then
        last_altitude_report_ms = now_ms
        gcs:send_text(6, string.format(
            "Circle alt rng=%.2f ref=%.2f ekf=%.2f tgt=%.2f dz=%+.2f q=%d st=%d",
            lidar_last_height_m,
            lidar_target_height_m,
            ekf_height_m,
            target_height_m,
            z_correction_m,
            lidar_quality,
            lidar_status))
    end
end

function circle()
    local cur_freq
    -- increase target speed lineary with time until ramp_up_time_s is reached
    if time <= ramp_up_time_s then
        cur_freq = omega_radps*(time/ramp_up_time_s)^2
    else
        cur_freq = omega_radps
    end

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
        if lidar_fault_latched then
            return update, 500
        end

        if not circle_active then
            return_mode_num = vehicle:get_mode()
            if not set_start_location() then
                gcs:send_text(4, "Circle: EKF position or downward LiDAR unavailable")
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

        if not update_lidar_height_control() then
            restore_return_mode("Circle: LiDAR timeout, restored previous mode")
            circle_active = false
            lidar_fault_latched = true
            time = 0
            theta = 0
            gcs:send_text(4, "Circle: Toggle CH9 low before retrying")
            return update, 500
        end

        -- calculate current position and velocity for circle trajectory
        local target_pos, target_vel = circle()
        target_pos:z(z_correction_m)
        target_vel:z(z_correction_rate_mps)

        -- advance the time
        time = time + sampling_time_s

        -- send posvel request
        local target_pos_ned = target_pos + test_start_location
        if not vehicle:set_target_posvel_NED(target_pos_ned, target_vel) then
            gcs:send_text(0, "Failed to send target posvel at " .. tostring(time) .. " seconds")
        end
        log_altitude(target_pos_ned:z())
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
        lidar_fault_latched = false
        last_altitude_report_ms = 0
    end

    return update, sampling_time_s * 1000
end

return update()
