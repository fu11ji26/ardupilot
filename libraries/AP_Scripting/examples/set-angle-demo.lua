-- Example of guided mode angle control, yawing back and forth

local target_yaw_1 = 0
local target_yaw_2 = 0

local target_roll_1 = 90 -- deg
local target_roll_2 = -90 -- deg
local target_pitch = 0 -- deg

local climb_rate = 0 -- m/s
local use_yaw_rate = false
local yaw_rate = 0 -- degs/s

local flipflop = true
function update()
  pwm6 = rc:get_pwm(6)
  if pwm6 and pwm6 > 1800 then    -- check if RC6 input has moved high
    -- if (vehicle:set_mode(27)) then  -- change to Starwars mode
    -- vehicle:set_mode(27)
    if flipflop then
      vehicle:set_target_angle_and_climbrate(target_roll_1,target_pitch,target_yaw_1,climb_rate,use_yaw_rate,yaw_rate)
    else 
      vehicle:set_target_angle_and_climbrate(target_roll_2,target_pitch,target_yaw_2,climb_rate,use_yaw_rate,yaw_rate)
    end
    flipflop = not flipflop
    return update, 5000
  end
end

return update()
