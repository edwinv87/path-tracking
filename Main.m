% Main file for Line Tracking 
%
%
%
%-------------------------------------------------------------------------%
close all;
clear;
clc;

% Load Camera Matrix
load kk;

% Load Optimal Fuzzy Parameters
load optimal;

% Load Extrinsic Parameters: Translation Vector and Rotation Matrix
load transrot;

% Constants and variables
alpha = 0.85;        % Error sensitivity factor
prev_eT = 0;        % Error at (t - 1) timestep
vel = 0.2;          % Linear velocity of the robot
phi = 0;            % Effective steering angle
eT = 0;


% Parameters of the robot
R = 0.0635;         % Radius of the robot wheels in meters
Wb = 0.328;         % Wheel base of the robot in meters
L = 0.3;            % Distance from the wheel center axis to caster center axis in meters
w_max = 1;
phi_max = atan((w_max*L)/vel);
c_max = tan(phi_max)/L;



global Y;           % Velocity vl, vr (vr first)
global t;           % Time stamp
Y = [];
t = [];
N = 1000;
crve = zeros(1,N);
eL_acq = zeros(1,N);
eH_acq = zeros(1,N);
phi_acq = zeros(1,N);
eL = 0;
eH = 0;
curve = 0;
vel_sf = 1;
load opt;

% Initialize Controller ---------------------------------------------------
import mbed.*;
period = 0.0005;    % Period of the PWM signal (2kHz)
controller = SerialRPC('COM6', 57600);

% Connection to right motor controller
pwm_r = PwmOut(controller, p21);
en_r = DigitalOut(controller, p22);
dir_r = DigitalOut(controller, p23);
stop_r = DigitalOut(controller, p24);

% Connection to left motor controller
pwm_l = PwmOut(controller, p25);
en_l = DigitalOut(controller, p26);
dir_l = DigitalOut(controller, p27);
stop_l = DigitalOut(controller, p28);

% Disable the ESCON power stage
en_r.write(0);
en_l.write(0);

% Set to running mode - Motors will stop if high
stop_l.write(0);
stop_r.write(0);

% Setting the period of the PWM Signal
pwm_l.period(period);
pwm_r.period(period);

% Initial duty cycle will always be 0.1f
pwm_l.write(0.1);
pwm_r.write(0.1);


% Initialize Video Input --------------------------------------------------
vid = videoinput('winvideo', 1, 'MJPG_640x480');
set(vid, 'FramesPerTrigger', 1);
set(vid, 'TriggerRepeat', Inf);
triggerconfig(vid,'manual');
start(vid);
disp('Video Input Created Successfully...');
% Initialize Data Acquisition ---------------------------------------------
s = daq.createSession('ni');
ch0 = s.addAnalogInputChannel('Dev1', 'ai0', 'Voltage');  % Left Wheel Velocity
ch1 = s.addAnalogInputChannel('Dev1', 'ai1', 'Voltage');  % Right Wheel Velocity
s.Rate = 10;       % 10 Hz
s.IsContinuous = true;
ch0.InputType = 'Differential';
ch1.InputType = 'Differential';
lh = s.addlistener('DataAvailable',@ProcessVelocityData);
s.startBackground();
disp('Data Acquisition Initialized...');

% Enable the ESCON power stage
en_r.write(1);
en_l.write(1);
% figure;
% colormap([0 0 0;1 1 1]);
% MAIN LOOP ---------------------------------------------------------------
for x = 1:N
    % Acquire a frame (640 x 480)
    trigger(vid);
    frame = getdata(vid);

    % Process the frame
    im_gs = rgb2gray(frame);
    im_bw = im2bw(im_gs, 0.7);
    % L = medfilt2(im_gs,[6 6]);
    % figure, imshow(L)
    % [~, threshold] = edge(L, 'sobel');
    % fudgeFactor = 2.5;
    % BWs = edge(im_gs,'sobel', 0.1, 'nothinning');
    % image(im_bw);
    % Extract the line
    line_pixel = ExtractMidpoints2(im_bw);

    if (~isempty(line_pixel))
        % Get data from extracted line
        [curve, eH, eL] = GetLineParameters(line_pixel, KK, Rc_ext, Tc_ext);

        % Compute error and change in error
        eT = alpha*eL + (1 - alpha)*eH; % Tracking Error
        delta_eT = eT - prev_eT;
        % Pass error to Fuzzy Controller
        phi_dot = FuzzyPI(eT,delta_eT, Pg);
        % phi = 4.5*eT;
        phi = phi + phi_dot;
        delta_phi = abs(phi) - phi_max;
        vel_sf = FuzzyDriftControl(delta_phi, (abs(curve)/c_max));
      
        % Pass output to steering module
        Wheel(vel*vel_sf, phi, pwm_l, dir_l, pwm_r, dir_r);
        % disp(curve);
        
            % Set error
        prev_eT = eT;
    end
    crve(x) = curve;
    eL_acq(x) = eL;
    eH_acq(x) = eH;
    phi_acq(x) = phi;
    clear frame;
    clear im_gs;
    % clear L;
    clear im_bw;
    % clear BWs;
    % pause(0.005);
    % gcf;
end
% End of Main Loop --------------------------------------------------------
% Stop and Disable ESCON Power
stop_l.write(1);
stop_r.write(1);
en_r.write(0);
en_l.write(0);

% Clean Up ----------------------------------------------------------------
% Delete controller object 
controller.reset;
controller.delete;

% Stop and delete image acquisition objects
stop(vid);
delete(vid);

% Stop and delete data acquisition
% Note: Save Data before clearing memory
save('vel_data','Y', 't');
s.stop()
delete(lh);
delete(s);


save('control_data', 'eH_acq', 'eL_acq', 'crve', 'phi_acq');
% Final CleanUp
delete(instrfind({'Port'},{'COM6'}));
clc;
clear;
close all;