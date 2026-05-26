-- 1.0m離陸 -> 8の字(長方形ベース)を飛行するスクリプト
-- NEDローカル座標を使って屋内向けに目標位置を指定する版
local takeoff_alt = 1.0
local ch9_threshold = 1500
local copter_guided_mode_num = 4
local acceptance_radius = 0.3
local post_takeoff_hold_ms = 1500
local min_corner_hold_ms = 300
local max_corner_hold_ms = 1200
local stop_speed_threshold = 0.20 -- 0.25
local height_report_interval_ms = 500

local move_dist_right = 7.0
local move_dist_forward = 5.0

-- { 前方[m], 右方向[m] } を離陸後の機体向き基準で指定
local moves = {
    { 2.5, 0 },
    { 0, -move_dist_right },
    { -move_dist_forward, 0 },
    { 0, move_dist_right },
    { move_dist_forward, 0 },
    { 0, move_dist_right },
    { -move_dist_forward, 0 },
    { 0, -move_dist_right },
    { move_dist_forward, 0 }
}



local stage = 0
local target_pos_ned = nil
local return_mode_num = nil
local move_index = 1
local hold_start_ms = 0
local targets = {}
local takeoff_reached_ms = 0
local start_pos_ned = nil
local start_yaw_rad = nil
local last_height_report_ms = 0
local last_diag_message = {}

local function copy_vector3f(src)
    local dst = Vector3f()
    dst:x(src:x())
    dst:y(src:y())
    dst:z(src:z())
    return dst
end

local function get_position_ned_m()
    return ahrs:get_relative_position_NED_origin()
end

local function horizontal_distance_m(pos1, pos2)
    local dx = pos1:x() - pos2:x()
    local dy = pos1:y() - pos2:y()
    return math.sqrt(dx * dx + dy * dy)
end

local function body_offset_to_ned(forward_m, right_m, yaw_rad)
    local cos_yaw = math.cos(yaw_rad)
    local sin_yaw = math.sin(yaw_rad)
    local north_m = forward_m * cos_yaw - right_m * sin_yaw
    local east_m = forward_m * sin_yaw + right_m * cos_yaw
    return north_m, east_m
end

local function wrap_pi(angle_rad)
    while angle_rad > math.pi do
        angle_rad = angle_rad - 2.0 * math.pi
    end
    while angle_rad < -math.pi do
        angle_rad = angle_rad + 2.0 * math.pi
    end
    return angle_rad
end

local function get_location_alt_frame(loc)
    if loc == nil then
        return "unknown"
    end
    if loc:relative_alt() then
        return "home"
    end
    if loc:terrain_alt() then
        return "terrain"
    end
    if loc:origin_alt() then
        return "origin"
    end
    return "abs"
end

local function report_current_height(force)
    local now = millis()
    if not force and last_height_report_ms ~= 0 and now - last_height_report_ms < height_report_interval_ms then
        return
    end
    last_height_report_ms = now

    local rotation_downward = 25 -- ROTATION_PITCH_270 (rotations.h) 下向きrangefinder
    local curr_loc = ahrs:get_location()
    local curr_pos_ned = get_position_ned_m()
    local curr_yaw_rad = ahrs:get_yaw_rad()

    local rng_msg = "rng=NA"
    if rangefinder:has_data_orient(rotation_downward) then
        rng_msg = string.format("rng=%.2fm", rangefinder:distance_orient(rotation_downward))
    end

    local ekf_msg = "ekf_h=NA"
    if curr_pos_ned ~= nil then
        ekf_msg = string.format("ekf_h=%.2fm", -curr_pos_ned:z())
    end

    local loc_msg = "loc_alt=NA"
    if curr_loc ~= nil then
        loc_msg = string.format("loc_alt=%.2fm(%s)", curr_loc:alt() * 0.01, get_location_alt_frame(curr_loc))
    end

    local yaw_msg = "yaw_drift=NA"
    if start_yaw_rad ~= nil and curr_yaw_rad ~= nil then
        local yaw_err_deg = math.deg(wrap_pi(curr_yaw_rad - start_yaw_rad))
        yaw_msg = string.format("yaw_drift=%.1fdeg", yaw_err_deg)
    end

    gcs:send_text(6, string.format("Figure8 NED height: %s %s %s %s", rng_msg, ekf_msg, loc_msg, yaw_msg))
end

local function send_diag_once(key, severity, message)
    if last_diag_message[key] == message then
        return
    end
    last_diag_message[key] = message
    gcs:send_text(severity, message)
end

local function clear_diag(key)
    last_diag_message[key] = nil
end

local function report_sensor_diagnostics()
    local rotation_downward = 25 -- ROTATION_PITCH_270 (rotations.h) 下向きrangefinder
    local curr_loc = ahrs:get_location()
    local rel_pos_ned = get_position_ned_m()
    local vel_ned = ahrs:get_velocity_NED()

    if rangefinder:has_data_orient(rotation_downward) then
        clear_diag("rangefinder")
    else
        send_diag_once("rangefinder", 4, "Figure8 NED diag: rangefinder data unavailable")
    end

    if curr_loc ~= nil then
        clear_diag("location")
    else
        send_diag_once("location", 4, "Figure8 NED diag: AHRS location unavailable")
    end

    if rel_pos_ned ~= nil then
        clear_diag("relative_ned")
    else
        send_diag_once("relative_ned", 4, "Figure8 NED diag: relative NED position unavailable")
    end

    if vel_ned ~= nil then
        clear_diag("velocity")
    else
        send_diag_once("velocity", 4, "Figure8 NED diag: NED velocity unavailable")
    end
end

local function restore_return_mode(reason)
    if return_mode_num == nil then
        return
    end

    if vehicle:set_mode(return_mode_num) then
        gcs:send_text(6, reason .. string.format(" (mode %d)", return_mode_num))
    else
        gcs:send_text(4, "Figure8 NED: Failed to restore previous mode")
    end
    return_mode_num = nil
end

local function build_targets(base_pos_ned, yaw_rad)
    if base_pos_ned == nil or yaw_rad == nil then
        return false
    end

    targets = {}
    local cursor = copy_vector3f(base_pos_ned)

    for i = 1, #moves do
        local north_m, east_m = body_offset_to_ned(moves[i][1], moves[i][2], yaw_rad)
        cursor:x(cursor:x() + north_m)
        cursor:y(cursor:y() + east_m)
        cursor:z(base_pos_ned:z())
        targets[i] = copy_vector3f(cursor)
    end

    return true
end

local function set_next_move_target()
    if move_index > #targets then
        restore_return_mode("Figure8 NED: Finished pattern. Restored previous mode")
        stage = 11
        return
    end

    local next_pos = targets[move_index]
    if next_pos == nil then
        gcs:send_text(4, "Figure8 NED: Missing target position")
        return
    end

    if vehicle:set_target_pos_NED(next_pos, false, 0, false, 0, false, false) then
        target_pos_ned = next_pos
        gcs:send_text(6, string.format("Figure8 NED: Moving step %d/%d", move_index, #targets))
        stage = 4
    else
        gcs:send_text(4, "Figure8 NED: Failed to set target position")
    end
end

function update()
    local ch9_pwm = rc:get_pwm(9)
    if not ch9_pwm then
        return update, 1000
    end

    if not arming:is_armed() or ch9_pwm < ch9_threshold then
        if stage ~= 0 then
            restore_return_mode("Figure8 NED: Restored previous mode")
            gcs:send_text(6, "Figure8 NED: Resetting sequence")
            stage = 0
        end
        target_pos_ned = nil
        start_pos_ned = nil
        start_yaw_rad = nil
        targets = {}
        hold_start_ms = 0
        takeoff_reached_ms = 0
        last_height_report_ms = 0
        last_diag_message = {}
        move_index = 1
        return update, 500
    end

    report_sensor_diagnostics()
    report_current_height(false)

    if stage == 0 then
        if return_mode_num == nil then
            return_mode_num = vehicle:get_mode()
        end
        if vehicle:set_mode(copter_guided_mode_num) then
            gcs:send_text(6, "Figure8 NED Stage0: Changed to GUIDED")
            stage = 1
        end

    elseif stage == 1 then
        gcs:send_text(6, "Figure8 NED Stage1: Taking off")
        if vehicle:start_takeoff(takeoff_alt) then
            stage = 2
        end

    elseif stage == 2 then
        local rotation_downward = 25 -- ROTATION_PITCH_270 (rotations.h) 下向きrangefinder
        if rangefinder:has_data_orient(rotation_downward) then
            local current_alt = rangefinder:distance_orient(rotation_downward)
            local alt_err = math.abs(current_alt - takeoff_alt)
            gcs:send_text(6, string.format("Figure8 NED alt err: %.2f", alt_err))
            report_current_height(true)
            if alt_err < 0.2 then
                if takeoff_reached_ms == 0 then
                    takeoff_reached_ms = millis()
                    gcs:send_text(6, "Figure8 NED Stage2: Reached takeoff altitude, holding")
                end
                if millis() - takeoff_reached_ms >= post_takeoff_hold_ms then
                    start_pos_ned = get_position_ned_m()
                    if start_pos_ned == nil then
                        gcs:send_text(4, "Figure8 NED: Position unavailable from EKF origin")
                        return update, 100
                    end
                    start_yaw_rad = ahrs:get_yaw_rad()
                    if start_yaw_rad == nil then
                        gcs:send_text(4, "Figure8 NED: Yaw unavailable")
                        return update, 100
                    end
                    if not build_targets(start_pos_ned, start_yaw_rad) then
                        gcs:send_text(4, "Figure8 NED: Failed to build targets")
                        return update, 100
                    end
                    takeoff_reached_ms = 0
                    gcs:send_text(6, "Figure8 NED Stage2: Takeoff hold complete")
                    stage = 3
                end
            else
                takeoff_reached_ms = 0
            end
        else
            gcs:send_text(6, "Figure8 NED Stage2: NO DATA")
        end

    elseif stage == 3 then
        set_next_move_target()

    elseif stage == 4 then
        local curr_pos_ned = get_position_ned_m()
        if curr_pos_ned and target_pos_ned then
            local dist = horizontal_distance_m(curr_pos_ned, target_pos_ned)
            if dist < acceptance_radius then
                gcs:send_text(6, string.format("Figure8 NED: Reached step %d/%d", move_index, #targets))
                hold_start_ms = millis()
                stage = 5
            end
        end

    elseif stage == 5 then
        local hold_ms = millis() - hold_start_ms
        local stopped = false
        local vel_ned = ahrs:get_velocity_NED()

        if vel_ned ~= nil then
            stopped = vel_ned:xy():length() < stop_speed_threshold
        end

        if (hold_ms >= min_corner_hold_ms and stopped) or (hold_ms >= max_corner_hold_ms) then
            move_index = move_index + 1
            stage = 3
        end

    elseif stage == 11 then
        -- 完了後は元のモードを維持
        return update, 100
    end

    return update, 100
end

return update()
