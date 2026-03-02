-- 飛行中にプロポのスイッチで8の字(長方形ベース)を実行するスクリプト

local rc_channel = 7             -- トリガーとなるプロポのチャンネル
local pwm_threshold = 1700       -- スイッチONと判定するPWM値
local fwd_dist = 6.0             -- 前後方向の移動距離 (m)
local side_dist = 9.0            -- 左右方向の移動距離 (m), ポール(7m)を越える設定
local acceptance_radius = 0.5    -- 目標地点に到達したと判定する半径 (m)
local timeout_ms = 15000         -- 各区間のタイムアウト時間 (15秒)

local copter_guided_mode = 4     -- GUIDEDモード
local copter_loiter_mode = 5     -- LOITERモード

-- { 前方向の移動量(m), 右方向の移動量(m) }
local moves = {
    { fwd_dist, 0 },             -- 1: 前 (ライン0 通過)
    { 0, -side_dist },           -- 2: 左 (ラインA側ポールを回避)
    { -fwd_dist, 0 },            -- 3: 後ろ (ラインA 通過)
    { 0, side_dist },            -- 4: 右 (中央へ戻る)
    { fwd_dist, 0 },             -- 5: 前 (ライン0 通過)
    { 0, side_dist },            -- 6: 右 (ラインB側ポールを回避)
    { -fwd_dist, 0 },            -- 7: 後ろ (ラインB 通過)
    { 0, -side_dist },           -- 8: 左 (中央へ戻る)
    { fwd_dist, 0 }              -- 9: 前 (ライン0 通過して完了)
}

local is_active = false
local current_step = 1
local target_loc = nil
local start_yaw = 0
local step_start_time = 0

function set_next_target()
    if current_step > #moves then
        is_active = false
        vehicle:set_mode(copter_loiter_mode)
        gcs:send_text(6, "Mission Complete. Switched to LOITER.")
        return
    end

    local fwd_m = moves[current_step][1]
    local rgt_m = moves[current_step][2]

    -- 機体基準の移動量を, 開始時のYaw角を使って地球基準(北/東)の移動量に回転変換
    local offset_north = fwd_m * math.cos(start_yaw) - rgt_m * math.sin(start_yaw)
    local offset_east  = fwd_m * math.sin(start_yaw) + rgt_m * math.cos(start_yaw)

    -- 目標座標を更新
    target_loc:offset(offset_north, offset_east)

    if vehicle:set_target_location(target_loc) then
        gcs:send_text(6, string.format("Moving to step %d/%d", current_step, #moves))
        step_start_time = millis()
    else
        gcs:send_text(4, "Failed to set target location")
    end
end

function update()
    local curr_loc = ahrs:get_location()
    if not curr_loc then return update, 100 end

    local pwm = rc:get_pwm(rc_channel)
    if not pwm then return update, 100 end

    local switch_on = pwm > pwm_threshold
    local is_armed = arming:is_armed()

    -- 飛行中(アーム済)にスイッチがONになった瞬間の開始処理
    if switch_on and is_armed and not is_active then
        if vehicle:set_mode(copter_guided_mode) then
            is_active = true
            current_step = 1
            start_yaw = ahrs:get_yaw()       -- 開始時の機首方位をロック
            target_loc = ahrs:get_location() -- 現在地を基準点としてロック
            
            gcs:send_text(6, "Figure-8 Sequence Started")
            set_next_target()
        else
            gcs:send_text(4, "Failed to enter GUIDED mode")
        end

    -- 実行中にスイッチがOFFにされた場合の中断処理
    elseif not switch_on and is_active then
        is_active = false
        vehicle:set_mode(copter_loiter_mode)
        gcs:send_text(6, "Aborted. Switched to LOITER.")
    end

    -- 実行中の到達判定
    if is_active then
        local dist = curr_loc:get_distance(target_loc)
        local elapsed = millis() - step_start_time
        
        -- 目標の半径内に到達したか, タイムアウト時間を過ぎたら次のステップへ
        if dist < acceptance_radius or elapsed > timeout_ms then
            current_step = current_step + 1
            set_next_target()
        end
    end

    return update, 100
end

gcs:send_text(6, "Figure-8 Script Loaded")
return update, 1000