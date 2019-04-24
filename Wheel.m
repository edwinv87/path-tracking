function Wheel(v, phi, pwm_l, dir_l, pwm_r, dir_r)
% This function computes the wheel velocities given inputs:
% v     - linear velocity of the robot in m/s
% phi   - steering angle of the robot w.r.t center axis in radians
%
% This function then uses the pwm and direction objects to set the values
% of PWM and direction on mbed LPC1768 which commands the ESCON controllers

% Parameters of the robot
R = 0.0635;       % Radius of the robot wheels in meters
Wb = 0.328;       % Wheel base of the robot in meters
L = 0.3;        % Distance from the wheel center axis to caster center axis in meters
RPM_MAX = 7000; % Maximum RPM of the MOTORS.

% Set Forward Directions

% Compute vr and vl
w = (v/L) * tan(phi); 
vR = (2*v - w*Wb)/2.0;
vL = (2*v + w*Wb)/2.0;

% Set the directions on ESCON 
if vR > 0
    dir_r.write(0);
else
    dir_r.write(1);
end

if vL > 0
    dir_l.write(1);
else
    dir_l.write(0);
end

% Then find the angular wheel velocities in rads/s
wL = abs(vL)/R;
wR = abs(vR)/R;

% Convert angular wheel velocities to Motor RPM
wL = ((wL/(2*pi))*60)*51; % Multiply wheel RPM by 51 to get Motor RPM
wR = ((wR/(2*pi))*60)*51;

% Set reference speed on ESCON using PWM
% RPM = 0, Duty Cycle = 0.1
% RPM = 7000, Duty Cycle = 0.9
duty_r = (wR/RPM_MAX)*0.8 + 0.1;
duty_l = (wL/RPM_MAX)*0.8 + 0.1;

if (duty_r > 0.9)
    duty_r = 0.9;
elseif (duty_r < 0.1)
    duty_r = 0.1;
end

if (duty_l > 0.9)
    duty_l = 0.9;
elseif (duty_l < 0.1)
    duty_l = 0.1;
end

pwm_l.write(duty_l);
pwm_r.write(duty_r);


end