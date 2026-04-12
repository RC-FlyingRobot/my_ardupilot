-- 1.0m離陸 -> 8の字(長方形ベース)を飛行するスクリプト
local takeoff_alt = 1.0
local move_dist = 2.0
local ch9_threshold = 1500
local copter_guided_mode_num = 4
local dummy_origin_lat = 354000000
local dummy_origin_lng = 1390000000
local dummy_origin_alt_m = 0
local acceptance_radius = 0.3

local move_dist_x = 9.5
local move_dist_y = 5.5


-- { North方向[m], East方向[m] }
local moves = {
    { 7.0, 0 },
    { 0, -move_dist_x },
    { -move_dist_y, 0 },
    { 0, move_dist_x },
    { move_dist_y, 0 },
    { 0, move_dist_x },
    { -move_dist_y, 0 },
    { 0, -move_dist_x },
    { move_dist_y, 0}
}

local stage = 0
local target_loc = nil
local origin_was_seeded = false
local return_mode_num = nil
local target_alt_cm = nil
local move_index = 1

local function seed_dummy_origin_if_needed()
    if ahrs:get_origin() then
        return true
    end

    local dummy_origin = Location()
    dummy_origin:lat(dummy_origin_lat)
    dummy_origin:lng(dummy_origin_lng)
    dummy_origin:set_alt_m(dummy_origin_alt_m, 0)

    if ahrs:set_origin(dummy_origin) then
        if not origin_was_seeded then
            gcs:send_text(6, "Square: Seeded dummy EKF origin")
            origin_was_seeded = true
        end
        return true
    end

    gcs:send_text(4, "Square: Failed to seed dummy EKF origin")
    return false
end

local function get_position_with_dummy_origin()
    local curr_loc = ahrs:get_location()
    if curr_loc then
        return curr_loc
    end

    if not seed_dummy_origin_if_needed() then
        return nil
    end

    return ahrs:get_location()
end

local function restore_return_mode(reason)
    if return_mode_num == nil then
        return
    end

    if vehicle:set_mode(return_mode_num) then
        gcs:send_text(6, reason .. string.format(" (mode %d)", return_mode_num))
    else
        gcs:send_text(4, "Square: Failed to restore previous mode")
    end
    return_mode_num = nil
end

local function apply_locked_altitude(loc)
    if loc == nil or target_alt_cm == nil then
        return
    end
    loc:alt(target_alt_cm)
end

local function set_next_move_target(base_loc)
    if move_index > #moves then
        restore_return_mode("Figure8: Finished pattern. Restored previous mode")
        stage = 11
        return
    end

    if base_loc == nil then
        gcs:send_text(6, "Figure8: Waiting for position")
        return
    end

    local next_loc = Location()
    next_loc:lat(base_loc:lat())
    next_loc:lng(base_loc:lng())
    next_loc:alt(base_loc:alt())
    next_loc:relative_alt(base_loc:relative_alt())
    next_loc:terrain_alt(base_loc:terrain_alt())
    next_loc:origin_alt(base_loc:origin_alt())
    next_loc:loiter_xtrack(base_loc:loiter_xtrack())

    next_loc:offset(moves[move_index][1], moves[move_index][2])
    apply_locked_altitude(next_loc)

    if vehicle:set_target_location(next_loc) then
        target_loc = next_loc
        gcs:send_text(6, string.format("Figure8: Moving step %d/%d", move_index, #moves))
        stage = 4
    else
        gcs:send_text(4, "Figure8: Failed to set target location")
    end
end

function update()
    local ch9_pwm = rc:get_pwm(9)
    if not ch9_pwm then
        return update, 1000
    end

    if not arming:is_armed() or ch9_pwm < ch9_threshold then
        if stage ~= 0 then
            restore_return_mode("Square: Restored previous mode")
            gcs:send_text(6, "Square: Resetting sequence")
            stage = 0
        end
        origin_was_seeded = false
        target_alt_cm = nil
        target_loc = nil
        move_index = 1
        return update, 500
    end

    if stage == 0 then
        if not ahrs:get_location() then
            seed_dummy_origin_if_needed()
        end
        if return_mode_num == nil then
            return_mode_num = vehicle:get_mode()
        end
        if vehicle:set_mode(copter_guided_mode_num) then
            gcs:send_text(6, "Figure8 Stage0: Changed to GUIDED")
            stage = 1
        end

    elseif stage == 1 then
        gcs:send_text(6, "Figure8 Stage1: Taking off")
        if vehicle:start_takeoff(takeoff_alt) then
            stage = 2
        end

    elseif stage == 2 then
        local rotation_downward = 25
        if rangefinder:has_data_orient(rotation_downward) then
            local current_alt = rangefinder:distance_orient(rotation_downward)
            local alt_err = math.abs(current_alt - takeoff_alt)
            gcs:send_text(6, string.format("Figure8 alt err: %.2f", alt_err))
            if alt_err < 0.2 then
                local lock_loc = get_position_with_dummy_origin()
                if lock_loc then
                    target_alt_cm = lock_loc:alt()
                    gcs:send_text(6, string.format("Figure8 locked altitude %.2fm", target_alt_cm * 0.01))
                end
                gcs:send_text(6, "Figure8 Stage2: Reached takeoff altitude")
                stage = 3
            end
        else
            gcs:send_text(6, "Figure8 Stage2: NO DATA")
        end

    elseif stage == 3 then
        local curr_loc = get_position_with_dummy_origin()
        set_next_move_target(curr_loc)

    elseif stage == 4 then
        local curr_loc = get_position_with_dummy_origin()
        if curr_loc and target_loc then
            local dist = curr_loc:get_distance(target_loc)
            if dist < acceptance_radius then
                gcs:send_text(6, string.format("Figure8: Reached step %d/%d", move_index, #moves))
                move_index = move_index + 1
                stage = 3
            end
        end

    elseif stage == 11 then
        -- 完了後は元のモードを維持
    end

    return update, 100
end

return update()

