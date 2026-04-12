-- 1.5m離陸 -> 前進2m -> 左移動2m -> 後退2m -> 右移動2m スクリプト
local takeoff_alt = 1.0
local move_dist = 2.0
local ch9_threshold = 1500
local copter_guided_mode_num = 4
local dummy_origin_lat = 354000000
local dummy_origin_lng = 1390000000
local dummy_origin_alt_m = 0

local stage = 0
local target_loc = nil
local origin_was_seeded = false
local return_mode_num = nil
local target_alt_cm = nil

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
            gcs:send_text(6, "Square Stage0: Changed to GUIDED")
            stage = 1
        end

    elseif stage == 1 then
        gcs:send_text(6, "Square Stage1: Taking off")
        if vehicle:start_takeoff(takeoff_alt) then
            stage = 2
        end

    elseif stage == 2 then
        local rotation_downward = 25
        if rangefinder:has_data_orient(rotation_downward) then
            local current_alt = rangefinder:distance_orient(rotation_downward)
            local alt_err = math.abs(current_alt - takeoff_alt)
            gcs:send_text(6, string.format("Square alt err: %.2f", alt_err))
            if alt_err < 0.2 then
                local lock_loc = get_position_with_dummy_origin()
                if lock_loc then
                    target_alt_cm = lock_loc:alt()
                    gcs:send_text(6, string.format("Square locked altitude %.2fm", target_alt_cm * 0.01))
                end
                gcs:send_text(6, "Square Stage2: Reached takeoff altitude")
                stage = 3
            end
        else
            gcs:send_text(6, "Square Stage2: NO DATA")
        end

    elseif stage == 3 then
        local curr_loc = get_position_with_dummy_origin()
        if curr_loc then
            curr_loc:offset(move_dist, 0)
            apply_locked_altitude(curr_loc)
            if vehicle:set_target_location(curr_loc) then
                target_loc = curr_loc
                gcs:send_text(6, "Square Stage3: Moving Forward 2m")
                stage = 4
            end
        else
            gcs:send_text(6, "Square Stage3: Waiting for position")
        end

    elseif stage == 4 then
        local curr_loc = get_position_with_dummy_origin()
        if curr_loc and target_loc then
            local dist = curr_loc:get_distance(target_loc)
            if dist < 0.3 then
                gcs:send_text(6, "Square Stage4: Reached forward target")
                stage = 5
            end
        end

    elseif stage == 5 then
        local curr_loc = get_position_with_dummy_origin()
        if curr_loc then
            curr_loc:offset(0, -move_dist)
            apply_locked_altitude(curr_loc)
            if vehicle:set_target_location(curr_loc) then
                target_loc = curr_loc
                gcs:send_text(6, "Square Stage5: Moving Left 2m")
                stage = 6
            end
        else
            gcs:send_text(6, "Square Stage5: Waiting for position")
        end

    elseif stage == 6 then
        local curr_loc = get_position_with_dummy_origin()
        if curr_loc and target_loc then
            local dist = curr_loc:get_distance(target_loc)
            if dist < 0.3 then
                gcs:send_text(6, "Square Stage6: Reached left target")
                stage = 7
            end
        end

    elseif stage == 7 then
        local curr_loc = get_position_with_dummy_origin()
        if curr_loc then
            curr_loc:offset(-move_dist, 0)
            apply_locked_altitude(curr_loc)
            if vehicle:set_target_location(curr_loc) then
                target_loc = curr_loc
                gcs:send_text(6, "Square Stage7: Moving Back 2m")
                stage = 8
            end
        else
            gcs:send_text(6, "Square Stage7: Waiting for position")
        end

    elseif stage == 8 then
        local curr_loc = get_position_with_dummy_origin()
        if curr_loc and target_loc then
            local dist = curr_loc:get_distance(target_loc)
            if dist < 0.3 then
                gcs:send_text(6, "Square Stage8: Reached back target")
                stage = 9
            end
        end

    elseif stage == 9 then
        local curr_loc = get_position_with_dummy_origin()
        if curr_loc then
            curr_loc:offset(0, move_dist)
            apply_locked_altitude(curr_loc)
            if vehicle:set_target_location(curr_loc) then
                target_loc = curr_loc
                gcs:send_text(6, "Square Stage9: Moving Right 2m")
                stage = 10
            end
        else
            gcs:send_text(6, "Square Stage9: Waiting for position")
        end

    elseif stage == 10 then
        local curr_loc = get_position_with_dummy_origin()
        if curr_loc and target_loc then
            local dist = curr_loc:get_distance(target_loc)
            if dist < 0.3 then
                restore_return_mode("Square Stage10: Finished square. Restored previous mode")
                stage = 11
            end
        end

    elseif stage == 11 then
        -- 完了後は元のモードを維持
    end

    return update, 100
end

return update()

