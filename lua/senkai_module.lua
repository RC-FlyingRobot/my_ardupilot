-- circle_trajectory module  (senkai.lua をモジュール化したもの)
-- 配置先: APM/scripts/modules/senkai_module.lua  ← require() の検索パス
-- 旋回軌道の位置・速度計算を提供する。
-- 使い方:
--   local circle = require("senkai_module")
--   circle.set_origin(ned_pos)   -- 旋回中心をセット (Vector3f, NED座標)
--   circle.reset()               -- theta をリセット
--   local pos, vel = circle.step()   -- 次ステップの絶対NED位置と速度
--   circle.revolution_s()        -- 1周に要する秒数

---@diagnostic disable: cast-local-type
---@diagnostic disable: redundant-parameter

local M = {}

M.rad_xy_m         = 1.5    -- circle radius [m]
M.target_speed_mps = 1.0    -- tangential speed [m/s]
M.sampling_time_s  = 0.05   -- step interval [s]  (= INTERVAL_MS / 1000)

local omega_radps  = M.target_speed_mps / M.rad_xy_m
local theta        = 0.0
local origin       = Vector3f()

-- 旋回中心をセット (NED座標, Vector3f)
function M.set_origin(ned_pos)
    origin:x(ned_pos:x())
    origin:y(ned_pos:y())
    origin:z(ned_pos:z())
end

-- theta をリセット (旋回開始時に呼ぶ)
function M.reset()
    theta = 0.0
end

-- 1ステップ進め、絶対NED位置と速度 (Vector3f, Vector3f) を返す
function M.step()
    theta = theta + omega_radps * M.sampling_time_s

    local th_s = math.sin(theta)
    local th_c = math.cos(theta)

    local rel_pos = Vector3f()
    rel_pos:x(M.rad_xy_m * th_s)
    rel_pos:y(-M.rad_xy_m * (th_c - 1))
    rel_pos:z(0)

    local vel = Vector3f()
    vel:x(omega_radps * M.rad_xy_m * th_c)
    vel:y(omega_radps * M.rad_xy_m * th_s)
    vel:z(0)

    return rel_pos + origin, vel
end

-- 1周に要する秒数を返す
function M.revolution_s()
    return 2 * math.pi / omega_radps
end

return M
