% analyze_blackbox.m
% Simple analysis script for Pico drone RAM blackbox logs.
%
% CSV format from firmware:
% t_us,rollUs,pitchUs,thrUs,yawUs,...
% p_cmd,q_cmd,r_cmd,p_rate,q_rate,r_rate,...
% rollOut,pitchOut,yawOut,...
% m0,m1,m2,m3,rollDeg,pitchDeg

clear; clc; close all;

%% ===== User settings =====
logFile = 'flight_log_102.csv';   % <-- change to your filename
MAX_THROTTLE_CMD = 0.50;      % same as in firmware

%% ===== Load data (position-based, no table names needed) =====
T = readtable(logFile);           % let MATLAB do its thing
A = table2array(T);               % convert to plain numeric matrix

% Column indices (fixed by firmware print order)
COL_T_US      = 1;
COL_ROLLUS    = 2;
COL_PITCHUS   = 3;
COL_THRUS     = 4;
COL_YAWUS     = 5;
COL_P_CMD     = 6;
COL_Q_CMD     = 7;
COL_R_CMD     = 8;
COL_P_RATE    = 9;
COL_Q_RATE    = 10;
COL_R_RATE    = 11;
COL_ROLLOUT   = 12;
COL_PITCHOUT  = 13;
COL_YAWOUT    = 14;
COL_M0        = 15;
COL_M1        = 16;
COL_M2        = 17;
COL_M3        = 18;
COL_ROLLDEG   = 19;
COL_PITCHDEG  = 20;

% Extract columns
t_us     = A(:, COL_T_US);
rollUs   = A(:, COL_ROLLUS);
pitchUs  = A(:, COL_PITCHUS);
thrUs    = A(:, COL_THRUS);
yawUs    = A(:, COL_YAWUS);

p_cmd    = A(:, COL_P_CMD);
q_cmd    = A(:, COL_Q_CMD);
r_cmd    = A(:, COL_R_CMD);

p_rate   = A(:, COL_P_RATE);
q_rate   = A(:, COL_Q_RATE);
r_rate   = A(:, COL_R_RATE);

rollOut  = A(:, COL_ROLLOUT);
pitchOut = A(:, COL_PITCHOUT);
yawOut   = A(:, COL_YAWOUT);

m0_raw   = A(:, COL_M0);
m1_raw   = A(:, COL_M1);
m2_raw   = A(:, COL_M2);
m3_raw   = A(:, COL_M3);

rollDeg  = A(:, COL_ROLLDEG);
pitchDeg = A(:, COL_PITCHDEG);

%% ===== Derived quantities =====

% Time in seconds, starting at t=0
t = (t_us - t_us(1)) * 1e-6;

% Motors as [0..1] (they were logged as 0..255)
m0 = double(m0_raw) / 255;
m1 = double(m1_raw) / 255;
m2 = double(m2_raw) / 255;
m3 = double(m3_raw) / 255;

% Approximate overall throttle (mean of motors)
throttleAvg = (m0 + m1 + m2 + m3) / 4;

%% ===== Basic stats =====
dt = diff(t);

fprintf('Samples: %d\n', numel(t));
if ~isempty(dt)
    fprintf('Mean dt: %.4f s (%.1f Hz)\n', mean(dt), 1/mean(dt));
else
    fprintf('Only one sample in log.\n');
end

%% ===== Plot RC inputs =====
figure('Name','RC Inputs','NumberTitle','off');
subplot(4,1,1);
plot(t, rollUs, 'LineWidth', 1); grid on;
ylabel('Roll (us)');
title('RC Inputs (stick commands)');

subplot(4,1,2);
plot(t, pitchUs, 'LineWidth', 1); grid on;
ylabel('Pitch (us)');

subplot(4,1,3);
plot(t, thrUs, 'LineWidth', 1); grid on;
ylabel('Throttle (us)');

subplot(4,1,4);
plot(t, yawUs, 'LineWidth', 1); grid on;
ylabel('Yaw (us)');
xlabel('Time (s)');

%% ===== Plot commanded vs measured rates =====
figure('Name','Rate Response','NumberTitle','off');

% Roll
subplot(3,1,1);
plot(t, p_cmd, 'LineWidth', 1.2); hold on;
plot(t, p_rate, '--', 'LineWidth', 1);
grid on;
ylabel('Roll rate (deg/s)');
title('Roll rate: commanded vs measured');
legend('p\_cmd','p\_rate','Location','best');

% Pitch
subplot(3,1,2);
plot(t, q_cmd, 'LineWidth', 1.2); hold on;
plot(t, q_rate, '--', 'LineWidth', 1);
grid on;
ylabel('Pitch rate (deg/s)');
title('Pitch rate: commanded vs measured');
legend('q\_cmd','q\_rate','Location','best');

% Yaw
subplot(3,1,3);
plot(t, r_cmd, 'LineWidth', 1.2); hold on;
plot(t, r_rate, '--', 'LineWidth', 1);
grid on;
ylabel('Yaw rate (deg/s)');
xlabel('Time (s)');
title('Yaw rate: commanded vs measured');
legend('r\_cmd','r\_rate','Location','best');

%% ===== Plot rate errors (cmd - measured) =====
p_err = p_cmd - p_rate;
q_err = q_cmd - q_rate;
r_err = r_cmd - r_rate;

figure('Name','Rate Errors','NumberTitle','off');
subplot(3,1,1);
plot(t, p_err, 'LineWidth', 1); grid on;
ylabel('Roll err (deg/s)');
title('Rate errors (cmd - measured)');

subplot(3,1,2);
plot(t, q_err, 'LineWidth', 1); grid on;
ylabel('Pitch err (deg/s)');

subplot(3,1,3);
plot(t, r_err, 'LineWidth', 1); grid on;
ylabel('Yaw err (deg/s)');
xlabel('Time (s)');

%% ===== Plot motor outputs and throttle cap =====
figure('Name','Motor Outputs','NumberTitle','off');
plot(t, m0, 'LineWidth', 1.1); hold on;
plot(t, m1, 'LineWidth', 1.1);
plot(t, m2, 'LineWidth', 1.1);
plot(t, m3, 'LineWidth', 1.1);

yline(MAX_THROTTLE_CMD, '--k', 'Max throttle cmd', 'LineWidth', 1);

grid on;
xlabel('Time (s)');
ylabel('Motor fraction [0..1]');
title('Motor outputs (after mixing)');
legend({'m0','m1','m2','m3','throttle cap'},'Location','best');

%% ===== Plot average throttle vs cap =====
figure('Name','Average Throttle','NumberTitle','off');
plot(t, throttleAvg, 'LineWidth', 1.2); hold on;
yline(MAX_THROTTLE_CMD, '--r', 'Max throttle cmd', 'LineWidth', 1);
grid on;
xlabel('Time (s)');
ylabel('Mean motor fraction');
title('Average motor output vs throttle cap');

%% ===== Plot attitude (roll/pitch) =====
figure('Name','Attitude','NumberTitle','off');
subplot(2,1,1);
plot(t, rollDeg, 'LineWidth', 1.2); grid on;
ylabel('Roll (deg)');
title('Estimated attitude');

subplot(2,1,2);
plot(t, pitchDeg, 'LineWidth', 1.2); grid on;
ylabel('Pitch (deg)');
xlabel('Time (s)');

%% ===== Quick text summary about saturation =====
satIdx = throttleAvg >= (0.95 * MAX_THROTTLE_CMD);
satPercent = 100 * nnz(satIdx) / numel(throttleAvg);

fprintf('Time with avg throttle >= 95%% of cap: %.1f%% of samples\n', satPercent);

%% ===== Angle-mode (outer loop) analysis =====
% Only meaningful when you were in BALANCE mode (AUX2 mid/high).
% Assumes firmware MAX_ROLL_ANGLE_DEG = MAX_PITCH_ANGLE_DEG = 45 deg.

MAX_ROLL_ANGLE_DEG  = 45;
MAX_PITCH_ANGLE_DEG = 45;

% Convert RC microseconds to normalized sticks (-1..+1)
rollStick  = (rollUs  - 1500) / 500;
pitchStick = (pitchUs - 1500) / 500;

% Saturate just in case
rollStick  = max(min(rollStick,  1), -1);
pitchStick = max(min(pitchStick, 1), -1);

% Commanded angles from sticks
rollAngleCmdDeg  = rollStick  * MAX_ROLL_ANGLE_DEG;
pitchAngleCmdDeg = pitchStick * MAX_PITCH_ANGLE_DEG;

% Angle errors (cmd - measured)
rollAngleErrDeg  = rollAngleCmdDeg  - rollDeg;
pitchAngleErrDeg = pitchAngleCmdDeg - pitchDeg;

% Plot commanded vs measured angles
figure('Name','Angle Response (Balance Mode)','NumberTitle','off');

subplot(2,1,1);
plot(t, rollAngleCmdDeg, 'LineWidth', 1.2); hold on;
plot(t, rollDeg, '--', 'LineWidth', 1.0);
grid on;
ylabel('Roll (deg)');
title('Roll angle: commanded vs measured');
legend('rollAngleCmd','rollDeg','Location','best');

subplot(2,1,2);
plot(t, pitchAngleCmdDeg, 'LineWidth', 1.2); hold on;
plot(t, pitchDeg, '--', 'LineWidth', 1.0);
grid on;
ylabel('Pitch (deg)');
xlabel('Time (s)');
title('Pitch angle: commanded vs measured');
legend('pitchAngleCmd','pitchDeg','Location','best');

% Optional: plot angle errors
figure('Name','Angle Errors','NumberTitle','off');
subplot(2,1,1);
plot(t, rollAngleErrDeg, 'LineWidth', 1); grid on;
ylabel('Roll angle err (deg)');
title('Angle errors (cmd - measured)');

subplot(2,1,2);
plot(t, pitchAngleErrDeg, 'LineWidth', 1); grid on;
ylabel('Pitch angle err (deg)');
xlabel('Time (s)');

