-- senkai.lua
-- Reusable circle trajectory module for ArduPilot Copter (posvel method).
-- Usage:
--   local senkai = require('senkai')
--   senkai.set_start_location()
--   senkai.reset()
--   local pos, vel = senkai.circle()
--   vehicle:set_target_posvel_NED(pos + senkai.get_start_loc(), vel)

---@diagnostic disable: cast-local-type
---@diagnostic disable: redundant-parameter

local M = {}

-- Parameters
local rad_xy_m            = 1.5
local target_speed_xy_mps = 1.0
local ramp_up_time_s      = 3.0
local sampling_time_s     = 0.05

-- Derived
local omega_radps = target_speed_xy_mps / rad_xy_m

-- Internal state
local theta = 0.0
local time_s = 0.0
local start_location = Vector3f()

-- Expose sampling_time_s so callers can schedule at the correct rate
M.sampling_time_s = sampling_time_s

--- Capture current NED position (relative to EKF origin) as the circle center.
--- @return boolean success
function M.set_start_location()
    local cur_pos_ned = ahrs:get_relative_position_NED_origin()
    if cur_pos_ned == nil then
        return false
    end

    start_location:x(cur_pos_ned:x())
    start_location:y(cur_pos_ned:y())
    start_location:z(cur_pos_ned:z())
    return true
end

--- Compute the next position/velocity step on the circle and advance internal
--- state (theta, time_s).  The time_s value is read first (for cur_freq),
--- then theta is updated, then time_s is incremented -- matching the original
--- external-loop semantics where time was advanced after circle().
--- @return Vector3f pos, Vector3f vel
function M.circle()
    local cur_freq
    -- increase target speed linearly with time until ramp_up_time_s is reached
    if time_s <= ramp_up_time_s then
        cur_freq = omega_radps * (time_s / ramp_up_time_s) ^ 2
    else
        cur_freq = omega_radps
    end

    -- calculate circle reference position and velocity
    theta = theta + cur_freq * sampling_time_s

    local th_s = math.sin(theta)
    local th_c = math.cos(theta)

    local pos = Vector3f()
    pos:x(rad_xy_m * th_s)
    pos:y(-rad_xy_m * (th_c - 1))
    pos:z(0)

    local vel = Vector3f()
    vel:x(cur_freq * rad_xy_m * th_c)
    vel:y(cur_freq * rad_xy_m * th_s)
    vel:z(0)

    -- advance the time (after computing, matching original semantics)
    time_s = time_s + sampling_time_s

    return pos, vel
end

--- Reset theta and time_s to zero.
function M.reset()
    theta  = 0.0
    time_s = 0.0
end

--- Return the stored start location vector.
--- @return Vector3f
function M.get_start_loc()
    return start_location
end

--- Return the current accumulated angle in radians (2*pi = one full lap).
--- @return number
function M.get_theta()
    return theta
end

return M
