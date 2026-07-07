定数
local rad_xy_m = 1.5           -- 8の字の基本サイズ（1.5m）
local target_speed_xy_mps = 1.0  --移動速度（1.0m/s）
local ramp_up_time_s = 3.0     -- 加速にかける時間（3s）
local sampling_time_s = 0.05   --プログラムが計算する間隔（0.05s）
local ch6_threshold = 1500     --プロポの6番スイッチの反応ライン
local HOVER_ALT_CM = 200       --ホバリング高度の予備設定

変数
local omega_radps = target_speed_xy_mps / rad_xy_m  
--角速度[rad/s](=v/r)
local copter_guided_mode_num = 4  --GUIDEDモードを表す内部番号（4番）
local theta = 0.0                 --角度θ
local time = 0.0                  --経過時間
local test_start_location = Vector3f() --スタート地点の座標（3次元ベクトル）
local return_mode_num = nil       --スイッチを切った時に「元のモード」に戻すための記憶用
local circle_active = false       --今、8の字飛行中かどうか（はい/いいえ）
local FIGURE_8_CH = 6             --見張るスイッチは6番


起動時の文字列の表示（開始合図と旋回周期（=2π/ω））
tostring()は計算した数値を文字列として出力できる関数
gcs:send_text(0,"Script started")
gcs:send_text(0,"Trajectory period: " .. tostring(2 * math.rad(180) / omega_radps))


local function restore_return_mode(reason)
    if return_mode_num == nil then   
        return
    end
   --戻るべきモードが記憶されていないなら、何もしないで終わる

    if vehicle:set_mode(return_mode_num) then
        gcs:send_text(6, reason .. string.format(" (mode %d)", return_mode_num))
    else
        gcs:send_text(4, "Failed to restore previous mode")
    end
    return_mode_num = nil
end
vehicle:set_mode()は機体のモードを()内の番号に変更する関数
・成功したらモードを変更したことを示す文章をPC上に表示（%dにはreturn_mode_num(モード番号)が入る）
・失敗したら失敗したことを示す文章を表示
".."は文字列を連結し、"6"は通常のお知らせ、"4"は警告といった重要度を表す番号


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
ahrs:get_relative_position_NED_origin()は基準点からのm座標を返す関数
test_start_locationに現在地の座標を代入


function circle()
    local cur_freq = omega_radps
    
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
cur_freq=ω[rad/s]
θ=θ+ωt
(x,y,z)=(rsinθ,-r(cosθ-1),0)
(Vx,Vy,Vz)=(rωcosθ,rωsinθ,0)
x^2+(y-r)^2=r^2


function update()

    local ch9_pwm = rc:get_pwm(9)
    if not ch9_pwm then
        return update, 1000
    end
   --プロポの9番スイッチの電波（PWM値）を受信し、もし電波がなければ1000ミリ秒（1秒）後にまた最初からやり直す

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