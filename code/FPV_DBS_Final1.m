function FPV_DBS_Final1
% =========================================================================
% AI-Enabled Hybrid LiPo–Li-ion Battery Management System
% for FPV Quadcopter Simulation
% =========================================================================
%
% Author      : Sagar Sant
% Roll Number : 23035010233
% Program     : B.Sc. (Hons) Data Science and Artificial Intelligence
% Institute   : Indian Institute of Technology Guwahati
%
% Description :
% This MATLAB implementation presents an AI-enabled hybrid battery
% management system for FPV quadcopters, integrating LiPo and Li-ion
% batteries for adaptive and efficient energy utilization. The system
% combines flight dynamics simulation, nonlinear battery modeling, and
% an intelligent rule-based expert system to perform real-time,
% context-aware battery selection under dynamic operating conditions.
%
% AI Capabilities and Key Features :
% - AI-enabled rule-based expert decision engine for real-time battery selection
% - Context-aware decision-making using multi-parameter inputs:
%   load current, state of charge (SOC), voltage, and temperature
% - Mission-aware control across flight phases:
%   takeoff, cruise, maneuvering, return-to-base, and landing
% - Priority-based decision hierarchy ensuring safety-critical handling
% - Predictive decision-making for early return-to-base based on SOC thresholds
% - Reserve energy management using LiPo for safe and controlled landing
% - Hysteresis-based switching to prevent rapid oscillations and ensure stability
% - Multi-constraint optimization balancing performance, safety, and efficiency
% - Autonomous emergency handling under critical energy conditions
% - Real-time telemetry and visualization for system monitoring and validation
%
% Inputs :
% - Flight profile (time, mission phase, dynamic load conditions)
% - Battery parameters (capacity, voltage characteristics, temperature)
% - Environmental factors (stochastic disturbances such as wind)
%
% Outputs :
% - Intelligent battery selection (LiPo / Li-ion) over time
% - State of charge (SOC) evolution for both batteries
% - Voltage and temperature profiles under dynamic load
% - Battery switching behavior and decision traces
% - Real-time telemetry dashboard for performance visualization
%
% =========================================================================

close all; clc;

% ============ SIMULATION PARAMETERS ============
dt = 1.0; 
total_time = 3600; % 3600s (60 min)
t = 0:dt:total_time;
n_steps = length(t);

% ============ BATTERY SPECIFICATIONS ============
lipo_capacity_Ah = 2.0; 
lipo_voltage_full = 12.6; 
lipo_voltage_cutoff = 9.0; 
lipo_voltage_min_switch = 10.49; 
lipo_temp_max = 45.0; % C - Maximum safe temperature
lipo_temp_critical = 50.0; % C - Critical temperature

% Li-ion 3S configuration
lion_capacity_Ah = 1.5; 
lion_cells = 3;
lion_voltage_full = 4.2 * lion_cells; % 12.6V
lion_voltage_cutoff = 2.8 * lion_cells; % 8.4V cutoff
lion_voltage_min_switch = 3.19 * lion_cells; % 9.57V switch threshold
lion_temp_max = 45.0; % C - Maximum safe temperature
lion_temp_critical = 60.0; % C - Critical temperature

% Note: These values are used in the simulation logic
% lipo_voltage_min_switch and lion_voltage_min_switch are used in battery selection
% lipo_temp_max, lipo_temp_critical, lion_temp_max, lion_temp_critical are used for warnings

% Return to base thresholds
return_to_base_soc = 10; % Return to base when combined SOC below 10%
emergency_landing_soc = 3; % Emergency landing when combined SOC below 3%

% ============ INITIALIZE VARIABLES ============
lipo_voltage = zeros(1, n_steps);
lion_voltage = zeros(1, n_steps);
lipo_capacity = zeros(1, n_steps);
lion_capacity = zeros(1, n_steps);
active_battery = zeros(1, n_steps);
load_current = zeros(1, n_steps);
lipo_temp = zeros(1, n_steps);
lion_temp = zeros(1, n_steps);
drone_x = zeros(1, n_steps);
drone_y = zeros(1, n_steps);
drone_z = zeros(1, n_steps);
drone_roll = zeros(1, n_steps);
drone_pitch = zeros(1, n_steps);
drone_yaw = zeros(1, n_steps);

% Set initial conditions
lipo_voltage(1) = lipo_voltage_full;
lion_voltage(1) = lion_voltage_full;
lipo_capacity(1) = lipo_capacity_Ah;
lion_capacity(1) = lion_capacity_Ah;
lipo_temp(1) = 25.0;
lion_temp(1) = 25.0;
drone_x(1) = 0;
drone_y(1) = 0;
drone_z(1) = 0;
drone_roll(1) = 0;
drone_pitch(1) = 0;
drone_yaw(1) = 0;

% Trackers
lipo_discharged = 0; %#ok<NASGU>
lion_discharged = 0; %#ok<NASGU>

% Mission phase tracking
mission_phase = 'NORMAL';
return_start_index = 0;
home_position = [0, 0];

% ============ PRE-GENERATE FLIGHT PROFILE ============
fprintf('GENERATING FLIGHT PROFILE...\n');

for i = 1:n_steps
    time = t(i);
    
    if time < 30 % Takeoff
        load_current(i) = 18 + 2*randn();
        drone_z(i) = time * 0.5;
        drone_pitch(i) = 5;
    elseif time < 90 % Climb
        load_current(i) = 14 + 3*sin(time/10) + randn();
        drone_z(i) = 30 + (time-30) * 0.2;
        drone_pitch(i) = 3;
        drone_y(i) = (time-30) * 0.5;
    elseif time < 400 % Cruise
        load_current(i) = 7 + 2*sin(time/20) + 0.5*randn();
        drone_z(i) = 50;
        drone_y(i) = 30 + (time-90) * 0.8;
        drone_yaw(i) = 2*sin(time/50);
    elseif time < 800 % Acrobatic 1 with turbulence
        base = 8;
    % Add turbulence factor
        turbulence = randn() * 3; % Random turbulence up to 3 degrees
    
    if mod(time, 30) < 8
        load_current(i) = base + 12 + randn();
        drone_roll(i) = 30 + turbulence;
        drone_pitch(i) = 20 + turbulence * 0.5;
    elseif mod(time, 30) < 15
        load_current(i) = base + 5 + randn();
        drone_roll(i) = -15 + turbulence;
        drone_pitch(i) = -10 + turbulence * 0.5;
    else
        load_current(i) = base + 2 + sin(time/15);
        drone_roll(i) = turbulence;
        drone_pitch(i) = turbulence * 0.5;
    end
        drone_x(i) = (time-400) * 0.4;
        drone_y(i) = 250 + (time-400) * 0.3;
        drone_z(i) = 50 + 10*sin(time/20) + turbulence * 0.2;

    elseif time < 1200 % Long cruise 1
        load_current(i) = 6 + 1.5*sin(time/25) + 0.3*randn();
        drone_x(i) = 200;
        drone_y(i) = 400 + (time-800) * 0.5;
        drone_z(i) = 40;
    elseif time < 1600 % Acrobatic 2
        load_current(i) = 9 + 4*sin(time/15) + randn();
        drone_x(i) = 250 + (time-1200) * 0.3;
        drone_y(i) = 600 + (time-1200) * 0.2;
        drone_z(i) = 30 + 15*sin(time/25);
        drone_roll(i) = 15*sin(time/10);
        drone_pitch(i) = 10*sin(time/12);
    elseif time < 2000 % Long cruise 2
        load_current(i) = 5.5 + 1*sin(time/30) + 0.2*randn();
        drone_x(i) = 350;
        drone_y(i) = 700 + (time-1600) * 0.4;
        drone_z(i) = 35;
    else % Extended flight
        load_current(i) = 8 + 2.5*sin(time/35) + randn();
        drone_x(i) = 400;
        drone_y(i) = 860;
        drone_z(i) = 25;
    end
    
    load_current(i) = max(0.5, load_current(i) + 0.5);
    
    % Temperature modeling
    if i > 1
        lipo_temp(i) = lipo_temp(i-1) + 0.015 * load_current(i) - 0.006 * (lipo_temp(i-1) - 25);
        lion_temp(i) = lion_temp(i-1) + 0.008 * load_current(i) - 0.006 * (lion_temp(i-1) - 25);
    end
end

load_current = movmean(load_current, 5);

% ============ STORE FLIGHT PROFILE IN TEMPORARY VARIABLES ============
% Store all the pre-generated data in temporary variables first
drone_x_pre = drone_x;
drone_y_pre = drone_y;
drone_z_pre = drone_z;
drone_roll_pre = drone_roll;
drone_pitch_pre = drone_pitch;
drone_yaw_pre = drone_yaw;
load_current_pre = load_current;

% ============ CREATE DASHBOARD ============
fprintf('DEPLOYING DASHBOARD...\n');

dashboard = figure('Name', 'FPV ELITE: Dual Battery System', ...
    'NumberTitle', 'off', ...
    'Position', [10, 10, 1900, 1000], ...
    'Color', [0.05, 0.05, 0.1], ...
    'MenuBar', 'none', ...
    'ToolBar', 'none', ...
    'Resize', 'off', ...
    'KeyPressFcn', @keyPressCallback);

% Store data
dashboard.UserData.t = t;
dashboard.UserData.load_current = load_current;
dashboard.UserData.lipo_voltage = lipo_voltage;
dashboard.UserData.lion_voltage = lion_voltage;
dashboard.UserData.lipo_capacity = lipo_capacity;
dashboard.UserData.lion_capacity = lion_capacity;
dashboard.UserData.active_battery = active_battery;
dashboard.UserData.lipo_temp = lipo_temp;
dashboard.UserData.lion_temp = lion_temp;
dashboard.UserData.drone_x = drone_x;
dashboard.UserData.drone_y = drone_y;
dashboard.UserData.drone_z = drone_z;
dashboard.UserData.drone_roll = drone_roll;
dashboard.UserData.drone_pitch = drone_pitch;
dashboard.UserData.drone_yaw = drone_yaw;
dashboard.UserData.drone_x_pre = drone_x_pre;
dashboard.UserData.drone_y_pre = drone_y_pre;
dashboard.UserData.drone_z_pre = drone_z_pre;
dashboard.UserData.drone_roll_pre = drone_roll_pre;
dashboard.UserData.drone_pitch_pre = drone_pitch_pre;
dashboard.UserData.drone_yaw_pre = drone_yaw_pre;
dashboard.UserData.load_current_pre = load_current_pre;
dashboard.UserData.lipo_discharged = 0;
dashboard.UserData.lion_discharged = 0;
dashboard.UserData.lipo_capacity_Ah = lipo_capacity_Ah;
dashboard.UserData.lion_capacity_Ah = lion_capacity_Ah;
dashboard.UserData.lipo_voltage_full = lipo_voltage_full;
dashboard.UserData.lion_voltage_full = lion_voltage_full;
dashboard.UserData.lipo_voltage_cutoff = lipo_voltage_cutoff;
dashboard.UserData.lion_voltage_cutoff = lion_voltage_cutoff;
dashboard.UserData.lipo_voltage_min_switch = lipo_voltage_min_switch; % Used in battery selection
dashboard.UserData.lion_voltage_min_switch = lion_voltage_min_switch; % Used in battery selection
dashboard.UserData.lipo_temp_max = lipo_temp_max;
dashboard.UserData.lion_temp_max = lion_temp_max;
dashboard.UserData.lipo_temp_critical = lipo_temp_critical;
dashboard.UserData.lion_temp_critical = lion_temp_critical;
dashboard.UserData.return_to_base_soc = return_to_base_soc;
dashboard.UserData.emergency_landing_soc = emergency_landing_soc;
dashboard.UserData.home_position = home_position;
dashboard.UserData.mission_phase = mission_phase;
dashboard.UserData.return_start_index = return_start_index;
dashboard.UserData.i = 1;
dashboard.UserData.running = false; % Start with mission not running
dashboard.UserData.dt = dt;
dashboard.UserData.n_steps = n_steps;
dashboard.UserData.lion_cells = lion_cells;

% ============ ADD WEATHER EFFECTS ============
fprintf('ADDING WEATHER EFFECTS...\n');

% Wind parameters
base_wind_speed = 2.5; % m/s constant wind
wind_direction = 45; % degrees (45 = northeast)
gust_intensity = 1.5; % gust strength
gust_frequency = 0.1; % how often gusts occur

% Create array to store wind speed over time
wind_speed_history = zeros(1, n_steps);

for i = 1:n_steps
    % Add wind effect to drone position
    time = t(i);
    
    % Start with base wind
    current_wind = base_wind_speed;
    
    % Random gusts - increase wind speed during gusts
    if rand() < gust_frequency * dt
        gust_strength = gust_intensity * (0.5 + rand());
        current_wind = base_wind_speed + gust_strength;
    end
    
    % Store current wind speed
    wind_speed_history(i) = current_wind;
    
    % Constant wind drift (using base wind for position, gusts add extra)
    wind_drift_x = base_wind_speed * cosd(wind_direction) * dt * (time/100);
    wind_drift_y = base_wind_speed * sind(wind_direction) * dt * (time/100);
    
    % Add gust effect to position
    if rand() < gust_frequency * dt
        gust_strength = gust_intensity * (0.5 + rand());
        gust_angle = rand() * 360;
        wind_drift_x = wind_drift_x + gust_strength * cosd(gust_angle) * dt;
        wind_drift_y = wind_drift_y + gust_strength * sind(gust_angle) * dt;
    end
    
    % Apply wind to original path
    drone_x(i) = drone_x(i) + wind_drift_x;
    drone_y(i) = drone_y(i) + wind_drift_y;
    
    % Wind affects current draw (harder to fly in wind)
    load_current(i) = load_current(i) * (1 + 0.05 * abs(current_wind/5));


 % Wind affects drone orientation (roll and pitch)
    drone_roll(i) = drone_roll(i) + 2*randn() * (current_wind/10); % Wind affects roll
    drone_pitch(i) = drone_pitch(i) + 2*randn() * (current_wind/10); % Wind affects pitch
end
% Store wind speed history in UserData (NOW dashboard exists)
dashboard.UserData.wind_speed_history = wind_speed_history;
dashboard.UserData.wind_speed = wind_speed_history(1);

% ============ TITLE ============
title_panel = uipanel('Parent', dashboard, ...
    'Position', [0.02, 0.94, 0.96, 0.05], ...
    'BackgroundColor', [0, 0, 0], ...
    'BorderType', 'none');

title_ax = axes('Parent', title_panel, 'Position', [0, 0, 1, 1], 'Color', 'none');
axis(title_ax, 'off');

text(0.5, 0.6, 'FPV QUADCOPTER DUAL BATTERY SYSTEM', ...
    'FontSize', 14, 'FontWeight', 'bold', 'Color', [0.2, 0.9, 0.9], ...
    'HorizontalAlignment', 'center', 'FontName', 'Arial Black');

text(0.5, 0.2, 'LIVE TELEMETRY REAL-TIME MONITORING CAPACITY TRACKING', ...
    'FontSize', 8, 'Color', [0.8, 0.6, 1], 'HorizontalAlignment', 'center');

% ============ LEFT COLUMN - BATTERIES ============
% LiPo Battery Panel
lipo_panel = uipanel('Parent', dashboard, ...
    'Title', ' LIPO BATTERY - 3S 11.1V 2000mAh ', ...
    'FontSize', 10, 'FontWeight', 'bold', ...
    'BackgroundColor', [0.08, 0.08, 0.15], ...
    'ForegroundColor', [0.3, 0.9, 1], ...
    'BorderType', 'line', 'BorderWidth', 1.5, ...
    'Position', [0.02, 0.62, 0.187, 0.27]);

% Voltage
uicontrol('Parent', lipo_panel, 'Style', 'text', ...
    'String', 'VOLTAGE:', 'Position', [15, 160, 60, 16], ...
    'FontSize', 8, 'FontWeight', 'bold', 'ForegroundColor', [0.8, 0.8, 1], ...
    'BackgroundColor', [0.08, 0.08, 0.15], 'HorizontalAlignment', 'left');
lipo_voltage_display = uicontrol('Parent', lipo_panel, 'Style', 'text', ...
    'String', '12.60 V', 'Position', [120, 158, 70, 20], ...
    'FontSize', 11, 'FontWeight', 'bold', 'ForegroundColor', [0.3, 1, 0.8], ...
    'BackgroundColor', [0.02, 0.02, 0.08], 'HorizontalAlignment', 'center', 'Tag', 'lipo_voltage');

% Capacity
uicontrol('Parent', lipo_panel, 'Style', 'text', ...
    'String', 'CAPACITY:', 'Position', [15, 130, 60, 16], ...
    'FontSize', 8, 'FontWeight', 'bold', 'ForegroundColor', [0.8, 0.8, 1], ...
    'BackgroundColor', [0.08, 0.08, 0.15], 'HorizontalAlignment', 'left');
lipo_capacity_display = uicontrol('Parent', lipo_panel, 'Style', 'text', ...
    'String', '2000 mAh', 'Position', [120, 128, 70, 20], ...
    'FontSize', 11, 'FontWeight', 'bold', 'ForegroundColor', [0.3, 1, 0.8], ...
    'BackgroundColor', [0.02, 0.02, 0.08], 'HorizontalAlignment', 'center', 'Tag', 'lipo_capacity');

% SOC
uicontrol('Parent', lipo_panel, 'Style', 'text', ...
    'String', 'SOC:', 'Position', [15, 100, 60, 16], ...
    'FontSize', 8, 'FontWeight', 'bold', 'ForegroundColor', [0.8, 0.8, 1], ...
    'BackgroundColor', [0.08, 0.08, 0.15], 'HorizontalAlignment', 'left');
lipo_soc_display = uicontrol('Parent', lipo_panel, 'Style', 'text', ...
    'String', '100.0 %', 'Position', [120, 98, 70, 20], ...
    'FontSize', 11, 'FontWeight', 'bold', 'ForegroundColor', [0.3, 1, 0.8], ...
    'BackgroundColor', [0.02, 0.02, 0.08], 'HorizontalAlignment', 'center', 'Tag', 'lipo_soc');

% Temperature with warning
uicontrol('Parent', lipo_panel, 'Style', 'text', ...
    'String', 'TEMP:', 'Position', [15, 70, 60, 16], ...
    'FontSize', 8, 'FontWeight', 'bold', 'ForegroundColor', [0.8, 0.8, 1], ...
    'BackgroundColor', [0.08, 0.08, 0.15], 'HorizontalAlignment', 'left');
lipo_temp_display = uicontrol('Parent', lipo_panel, 'Style', 'text', ...
    'String', '25.0 C', 'Position', [70, 68, 60, 20], ...
    'FontSize', 11, 'FontWeight', 'bold', 'ForegroundColor', [0.3, 1, 0.8], ...
    'BackgroundColor', [0.02, 0.02, 0.08], 'HorizontalAlignment', 'center', 'Tag', 'lipo_temp');

lipo_temp_warning = uicontrol('Parent', lipo_panel, 'Style', 'text', ...
    'String', '', 'Position', [145, 65, 85, 22], ...
    'FontSize', 9, 'FontWeight', 'bold', 'ForegroundColor', [1, 0.5, 0], ...
    'BackgroundColor', [0.08, 0.08, 0.15], 'HorizontalAlignment', 'left', 'Tag', 'lipo_temp_warning');

% Progress bar
uicontrol('Parent', lipo_panel, 'Style', 'text', ...
    'String', '', 'Position', [15, 30, 220, 12], ...
    'BackgroundColor', [0.3, 0.3, 0.4], 'HorizontalAlignment', 'left');
lipo_bar = uicontrol('Parent', lipo_panel, 'Style', 'text', ...
    'String', '', 'Position', [15, 30, 220, 12], ...
    'FontSize', 8, 'ForegroundColor', [0, 1, 0.6], ...
    'BackgroundColor', [0.08, 0.08, 0.15], 'HorizontalAlignment', 'left', 'Tag', 'lipo_bar');

% Li-ion Battery Panel
lion_panel = uipanel('Parent', dashboard, ...
    'Title', ' LI-ION BATTERY - 3S 18650 11.1V 1500mAh ', ...
    'FontSize', 10, 'FontWeight', 'bold', ...
    'BackgroundColor', [0.08, 0.08, 0.15], ...
    'ForegroundColor', [1, 0.5, 0.8], ...
    'BorderType', 'line', 'BorderWidth', 1.5, ...
    'Position', [0.02, 0.30, 0.187, 0.27]);

% Voltage
uicontrol('Parent', lion_panel, 'Style', 'text', ...
    'String', 'VOLTAGE:', 'Position', [15, 160, 60, 16], ...
    'FontSize', 8, 'FontWeight', 'bold', 'ForegroundColor', [1, 0.8, 0.9], ...
    'BackgroundColor', [0.08, 0.08, 0.15], 'HorizontalAlignment', 'left');
lion_voltage_display = uicontrol('Parent', lion_panel, 'Style', 'text', ...
    'String', '12.60 V', 'Position', [120, 158, 70, 20], ...
    'FontSize', 11, 'FontWeight', 'bold', 'ForegroundColor', [1, 0.6, 0.8], ...
    'BackgroundColor', [0.02, 0.02, 0.08], 'HorizontalAlignment', 'center', 'Tag', 'lion_voltage');

% Capacity
uicontrol('Parent', lion_panel, 'Style', 'text', ...
    'String', 'CAPACITY:', 'Position', [15, 130, 60, 16], ...
    'FontSize', 8, 'FontWeight', 'bold', 'ForegroundColor', [1, 0.8, 0.9], ...
    'BackgroundColor', [0.08, 0.08, 0.15], 'HorizontalAlignment', 'left');
lion_capacity_display = uicontrol('Parent', lion_panel, 'Style', 'text', ...
    'String', '1500 mAh', 'Position', [120, 128, 70, 20], ...
    'FontSize', 11, 'FontWeight', 'bold', 'ForegroundColor', [1, 0.6, 0.8], ...
    'BackgroundColor', [0.02, 0.02, 0.08], 'HorizontalAlignment', 'center', 'Tag', 'lion_capacity');

% SOC
uicontrol('Parent', lion_panel, 'Style', 'text', ...
    'String', 'SOC:', 'Position', [15, 100, 60, 16], ...
    'FontSize', 8, 'FontWeight', 'bold', 'ForegroundColor', [1, 0.8, 0.9], ...
    'BackgroundColor', [0.08, 0.08, 0.15], 'HorizontalAlignment', 'left');
lion_soc_display = uicontrol('Parent', lion_panel, 'Style', 'text', ...
    'String', '100.0 %', 'Position', [120, 98, 70, 20], ...
    'FontSize', 11, 'FontWeight', 'bold', 'ForegroundColor', [1, 0.6, 0.8], ...
    'BackgroundColor', [0.02, 0.02, 0.08], 'HorizontalAlignment', 'center', 'Tag', 'lion_soc');

% Temperature with 
uicontrol('Parent', lion_panel, 'Style', 'text', ...
    'String', 'TEMP:', 'Position', [15, 70, 60, 16], ...
    'FontSize', 8, 'FontWeight', 'bold', 'ForegroundColor', [1, 0.8, 0.9], ...
    'BackgroundColor', [0.08, 0.08, 0.15], 'HorizontalAlignment', 'left');
lion_temp_display = uicontrol('Parent', lion_panel, 'Style', 'text', ...
    'String', '25.0 C', 'Position', [70, 68, 60, 20], ...
    'FontSize', 11, 'FontWeight', 'bold', 'ForegroundColor', [1, 0.6, 0.8], ...
    'BackgroundColor', [0.02, 0.02, 0.08], 'HorizontalAlignment', 'center', 'Tag', 'lion_temp');

lion_temp_warning = uicontrol('Parent', lion_panel, 'Style', 'text', ...
    'String', '', 'Position', [145, 65, 85, 22], ...
    'FontSize', 9, 'FontWeight', 'bold', 'ForegroundColor', [1, 0.5, 0], ...
    'BackgroundColor', [0.08, 0.08, 0.15], 'HorizontalAlignment', 'left', 'Tag', 'lion_temp_warning');

% Progress bar
uicontrol('Parent', lion_panel, 'Style', 'text', ...
    'String', '', 'Position', [15, 30, 220, 12], ...
    'BackgroundColor', [0.3, 0.3, 0.4], 'HorizontalAlignment', 'left');
lion_bar = uicontrol('Parent', lion_panel, 'Style', 'text', ...
    'String', '', 'Position', [15, 30, 220, 12], ...
    'FontSize', 8, 'ForegroundColor', [1, 0.4, 0.7], ...
    'BackgroundColor', [0.08, 0.08, 0.15], 'HorizontalAlignment', 'left', 'Tag', 'lion_bar');

% ============ FLIGHT INSTRUMENTS PANEL ============
flight_panel = uipanel('Parent', dashboard, ...
    'Title', ' FLIGHT INSTRUMENTS - LIVE TELEMETRY ', ...
    'FontSize', 11, 'FontWeight', 'bold', ...
    'BackgroundColor', [0.07, 0.07, 0.13], ...
    'ForegroundColor', [1, 1, 0.2], ...
    'BorderType', 'line', 'BorderWidth', 1.5, ...
    'Position', [0.27, 0.61, 0.27, 0.30]);

% Current draw
uicontrol('Parent', flight_panel, 'Style', 'text', ...
    'String', 'CURRENT DRAW', 'Position', [15, 190, 120, 18], ...
    'FontSize', 10, 'FontWeight', 'bold', 'ForegroundColor', [1, 1, 0.7], ...
    'BackgroundColor', [0.07, 0.07, 0.13], 'HorizontalAlignment', 'left');
current_display = uicontrol('Parent', flight_panel, 'Style', 'text', ...
    'String', '0.0 A', 'Position', [150, 180, 90, 42], ...
    'FontSize', 24, 'FontWeight', 'bold', 'ForegroundColor', [0.3, 1, 0.3], ...
    'BackgroundColor', [0, 0, 0], 'HorizontalAlignment', 'center', 'Tag', 'current');

% Active battery
uicontrol('Parent', flight_panel, 'Style', 'text', ...
    'String', 'ACTIVE BATTERY', 'Position', [15, 155, 120, 18], ...
    'FontSize', 9, 'FontWeight', 'bold', 'ForegroundColor', [1, 1, 0.7], ...
    'BackgroundColor', [0.07, 0.07, 0.13], 'HorizontalAlignment', 'left');
active_display = uicontrol('Parent', flight_panel, 'Style', 'text', ...
    'String', 'LIPO', 'Position', [150, 150, 130, 25], ...
    'FontSize', 14, 'FontWeight', 'bold', 'ForegroundColor', [0.3, 1, 0.8], ...
    'BackgroundColor', [0.02, 0.02, 0.08], 'HorizontalAlignment', 'center', 'Tag', 'active');

% Selected voltage
uicontrol('Parent', flight_panel, 'Style', 'text', ...
    'String', 'SELECTED VOLTAGE', 'Position', [15, 120, 130, 18], ...
    'FontSize', 9, 'FontWeight', 'bold', 'ForegroundColor', [1, 1, 0.7], ...
    'BackgroundColor', [0.07, 0.07, 0.13], 'HorizontalAlignment', 'left');
voltage_display = uicontrol('Parent', flight_panel, 'Style', 'text', ...
    'String', '12.6 V', 'Position', [150, 115, 80, 25], ...
    'FontSize', 14, 'FontWeight', 'bold', 'ForegroundColor', [1, 1, 0.3], ...
    'BackgroundColor', [0.02, 0.02, 0.08], 'HorizontalAlignment', 'center', 'Tag', 'voltage');

% Flight time
uicontrol('Parent', flight_panel, 'Style', 'text', ...
    'String', 'FLIGHT TIME', 'Position', [15, 85, 120, 18], ...
    'FontSize', 9, 'FontWeight', 'bold', 'ForegroundColor', [1, 1, 0.7], ...
    'BackgroundColor', [0.07, 0.07, 0.13], 'HorizontalAlignment', 'left');
time_display = uicontrol('Parent', flight_panel, 'Style', 'text', ...
    'String', '0:00', 'Position', [150, 75, 100, 35], ...
    'FontSize', 20, 'FontWeight', 'bold', 'ForegroundColor', [1, 1, 1], ...
    'BackgroundColor', [0, 0, 0], 'HorizontalAlignment', 'center', 'Tag', 'time');

% Status text
status_text = uicontrol('Parent', flight_panel, 'Style', 'text', ...
    'String', 'READY - PRESS START', 'Position', [5, 15, 400, 25], ...
    'FontSize', 9, 'FontWeight', 'bold', 'ForegroundColor', [0.8, 1, 0.8], ...
    'BackgroundColor', [0.07, 0.07, 0.13], 'HorizontalAlignment', 'center', 'Tag', 'status');

% Weather indicator
uicontrol('Parent', flight_panel, 'Style', 'text', ...
    'String', 'WEATHER:', 'Position', [15, 50, 70, 18], ...
    'FontSize', 8, 'FontWeight', 'bold', 'ForegroundColor', [0.5, 0.8, 1], ...
    'BackgroundColor', [0.07, 0.07, 0.13], 'HorizontalAlignment', 'left');
weather_display = uicontrol('Parent', flight_panel, 'Style', 'text', ...
    'String', 'CALM', 'Position', [150, 50, 80, 18], ...
    'FontSize', 9, 'FontWeight', 'bold', 'ForegroundColor', [0.5, 1, 0.5], ...
    'BackgroundColor', [0.02, 0.02, 0.08], 'HorizontalAlignment', 'center', 'Tag', 'weather');

% ============ 3D DRONE VISUALIZATION ============
drone_panel = uipanel('Parent', dashboard, ...
    'Title', ' 3D DRONE VISUALIZATION - LIVE FEED ', ...
    'FontSize', 11, 'FontWeight', 'bold', ...
    'BackgroundColor', [0.06, 0.06, 0.12], ...
    'ForegroundColor', [0.2, 1, 0.8], ...
    'BorderType', 'line', 'BorderWidth', 1.5, ...
    'Position', [0.21, 0.25, 0.38, 0.35]);

drone_ax = axes('Parent', drone_panel, ...
    'Position', [0.1, 0.1, 0.8, 0.8], ...
    'Color', [0.02, 0.02, 0.06], ...
    'XColor', [0.5, 0.5, 0.5], ...
    'YColor', [0.5, 0.5, 0.5], ...
    'ZColor', [0.5, 0.5, 0.5], ...
    'GridColor', [0.3, 0.8, 0.8], ...
    'GridAlpha', 0.2, ...
    'Box', 'on');
xlabel(drone_ax, 'X (m)', 'Color', [0.8, 0.8, 0.8], 'FontSize', 8);
ylabel(drone_ax, 'Y (m)', 'Color', [0.8, 0.8, 0.8], 'FontSize', 8);
zlabel(drone_ax, 'Z (m)', 'Color', [0.8, 0.8, 0.8], 'FontSize', 8);
title(drone_ax, 'FPV RACE COURSE', 'Color', [0.2, 1, 0.8], 'FontSize', 10);
grid(drone_ax, 'on');
hold(drone_ax, 'on');
view(drone_ax, 3);
axis(drone_ax, [-50, 500, -50, 1000, 0, 80]);

% Initialize drone
drone_data = createDrone(drone_ax);
drone_ax.UserData = drone_data;
drone_ax.Tag = 'drone_ax';

% ============ RIGHT COLUMN - PLOTS ============
plots_panel = uipanel('Parent', dashboard, ...
    'Title', ' REAL-TIME TELEMETRY - ENHANCED MONITORING ', ...
    'FontSize', 11, 'FontWeight', 'bold', ...
    'BackgroundColor', [0.07, 0.07, 0.13], ...
    'ForegroundColor', [0.2, 1, 0.8], ...
    'BorderType', 'line', 'BorderWidth', 1.5, ...
    'Position', [0.60, 0.25, 0.38, 0.62]);

% Current plot
current_ax = axes('Parent', plots_panel, ...
    'Position', [0.1, 0.78, 0.85, 0.18], ...
    'Color', [0.02, 0.02, 0.08], ...
    'XColor', [0.6, 0.6, 0.6], 'YColor', [0.6, 0.6, 0.6], ...
    'GridColor', [0.3, 0.9, 0.9], 'GridAlpha', 0.2, 'Box', 'on');
title(current_ax, 'LOAD CURRENT', 'Color', [0.3, 1, 0.8], 'FontSize', 9);
ylabel(current_ax, 'Current (A)', 'Color', [0.8, 0.8, 0.8], 'FontSize', 8);
grid(current_ax, 'on'); hold(current_ax, 'on');
xlim(current_ax, [0, total_time]);
ylim(current_ax, [0, 25]);
current_line = plot(current_ax, t(1:1), load_current(1:1), 'g-', 'LineWidth', 2);
plot(current_ax,[0 total_time],[8 8],'--','Color',[1 0.5 0.5],'LineWidth',1)
text(total_time-150,8.5,'Switch','Parent',current_ax,'Color',[1 0.5 0.5],'FontSize',8)

% Voltage plot
voltage_ax = axes('Parent', plots_panel, ...
    'Position', [0.1, 0.535, 0.85, 0.18], ...
    'Color', [0.02, 0.02, 0.08], ...
    'XColor', [0.6, 0.6, 0.6], 'YColor', [0.6, 0.6, 0.6], ...
    'GridColor', [1, 0.8, 0.2], 'GridAlpha', 0.2, 'Box', 'on');
title(voltage_ax, 'BATTERY VOLTAGES', 'Color', [1, 0.8, 0.2], 'FontSize', 9);
ylabel(voltage_ax, 'Voltage (V)', 'Color', [0.8, 0.8, 0.8], 'FontSize', 8);
grid(voltage_ax, 'on'); hold(voltage_ax, 'on');
xlim(voltage_ax, [0, total_time]);
ylim(voltage_ax, [7, 13]);
lipo_voltage_line = plot(voltage_ax, t(1:1), lipo_voltage(1:1), 'c-', 'LineWidth', 2);
lion_voltage_line = plot(voltage_ax, t(1:1), lion_voltage(1:1), 'm-', 'LineWidth', 2);
plot(voltage_ax,[0 total_time],[lipo_voltage_cutoff lipo_voltage_cutoff],'c--','LineWidth',1)
plot(voltage_ax,[0 total_time],[lion_voltage_cutoff lion_voltage_cutoff],'m--','LineWidth',1)
legend(voltage_ax, {'LiPo', 'Li-ion'}, 'TextColor', [1, 1, 1], ...
    'Color', [0.1, 0.1, 0.2], 'FontSize', 7, 'Box', 'off', 'Location', 'east');

% SOC plot
soc_ax = axes('Parent', plots_panel, ...
    'Position', [0.1, 0.29, 0.85, 0.18], ...
    'Color', [0.02, 0.02, 0.08], ...
    'XColor', [0.6, 0.6, 0.6], 'YColor', [0.6, 0.6, 0.6], ...
    'GridColor', [0.3, 1, 0.3], 'GridAlpha', 0.2, 'Box', 'on');
title(soc_ax, 'STATE OF CHARGE (SOC)', 'Color', [0.3, 1, 0.3], 'FontSize', 9);
ylabel(soc_ax, 'SOC (%)', 'Color', [0.8, 0.8, 0.8], 'FontSize', 8);
grid(soc_ax, 'on'); hold(soc_ax, 'on');
xlim(soc_ax, [0, total_time]);
ylim(soc_ax, [0, 105]);
lipo_soc_line = plot(soc_ax, t(1:1), (lipo_capacity(1:1)/lipo_capacity_Ah)*100, 'c-', 'LineWidth', 2);
lion_soc_line = plot(soc_ax, t(1:1), (lion_capacity(1:1)/lion_capacity_Ah)*100, 'm-', 'LineWidth', 2);
total_soc_line = plot(soc_ax, t(1:1), ((lipo_capacity(1:1)+lion_capacity(1:1))/(lipo_capacity_Ah+lion_capacity_Ah))*100, 'w-', 'LineWidth', 2);
legend(soc_ax, {'LiPo', 'Li-ion', 'Combined'}, 'TextColor', [1, 1, 1], ...
    'Color', [0.1, 0.1, 0.2], 'FontSize', 7, 'Box', 'off', 'Location', 'east');

% Temperature plot
temp_ax = axes('Parent', plots_panel, ...
    'Position', [0.1, 0.05, 0.85, 0.18], ...
    'Color', [0.02, 0.02, 0.08], ...
    'XColor', [0.6, 0.6, 0.6], 'YColor', [0.6, 0.6, 0.6], ...
    'GridColor', [1, 0.6, 0.2], 'GridAlpha', 0.2, 'Box', 'on');
title(temp_ax, 'BATTERY TEMPERATURE', 'Color', [1, 0.6, 0.2], 'FontSize', 9);
xlabel(temp_ax, 'Time (s)', 'Color', [0.8, 0.8, 0.8], 'FontSize', 8);
ylabel(temp_ax, 'Temperature (C)', 'Color', [0.8, 0.8, 0.8], 'FontSize', 8);
grid(temp_ax, 'on'); hold(temp_ax, 'on');
xlim(temp_ax, [0, total_time]);
ylim(temp_ax, [20, 60]);
lipo_temp_line = plot(temp_ax, t(1:1), lipo_temp(1:1), 'c-', 'LineWidth', 2);
lion_temp_line = plot(temp_ax, t(1:1), lion_temp(1:1), 'm-', 'LineWidth', 2);
plot(temp_ax,[0 total_time],[lipo_temp_max lipo_temp_max],'c--','LineWidth',1)
plot(temp_ax,[0 total_time],[lion_temp_max lion_temp_max],'m--','LineWidth',1)
legend(temp_ax, {'LiPo', 'Li-ion'}, 'TextColor', [1, 1, 1], ...
    'Color', [0.1, 0.1, 0.2], 'FontSize', 7, 'Box', 'off', 'Location', 'east');

% ============ FLIGHT TIME ESTIMATION PANEL ============
flight_time_panel = uipanel('Parent', dashboard, ...
    'Title', ' FLIGHT TIME ESTIMATION ', ...
    'FontSize', 11, 'FontWeight', 'bold', ...
    'BackgroundColor', [0.08, 0.08, 0.15], ...
    'ForegroundColor', [0.3, 1, 0.6], ...
    'BorderType', 'line', 'BorderWidth', 1.5, ...
    'Position', [0.02, 0.08, 0.37, 0.12]);

% Total capacity
uicontrol('Parent', flight_time_panel, 'Style', 'text', ...
    'String', 'TOTAL CAPACITY:', 'Position', [50, 40, 100, 16], ...
    'FontSize', 8, 'FontWeight', 'bold', 'ForegroundColor', [0.8, 1, 0.8], ...
    'BackgroundColor', [0.08, 0.08, 0.15], 'HorizontalAlignment', 'left');
total_capacity_display = uicontrol('Parent', flight_time_panel, 'Style', 'text', ...
    'String', '3500 mAh', 'Position', [190, 38, 70, 18], ...
    'FontSize', 9, 'FontWeight', 'bold', 'ForegroundColor', [0.3, 1, 0.6], ...
    'BackgroundColor', [0.02, 0.02, 0.08], 'HorizontalAlignment', 'center', 'Tag', 'total_capacity');

% Remaining capacity
uicontrol('Parent', flight_time_panel, 'Style', 'text', ...
    'String', 'REMAINING:', 'Position', [50, 20, 100, 16], ...
    'FontSize', 8, 'FontWeight', 'bold', 'ForegroundColor', [0.8, 1, 0.8], ...
    'BackgroundColor', [0.08, 0.08, 0.15], 'HorizontalAlignment', 'left');
remaining_capacity_display = uicontrol('Parent', flight_time_panel, 'Style', 'text', ...
    'String', '3500 mAh', 'Position', [190, 18, 70, 18], ...
    'FontSize', 9, 'FontWeight', 'bold', 'ForegroundColor', [1, 1, 0.3], ...
    'BackgroundColor', [0.02, 0.02, 0.08], 'HorizontalAlignment', 'center', 'Tag', 'remaining_capacity');

% Estimated total time
uicontrol('Parent', flight_time_panel, 'Style', 'text', ...
    'String', 'ESTIMATED TOTAL:', 'Position', [340, 40, 100, 16], ...
    'FontSize', 8, 'FontWeight', 'bold', 'ForegroundColor', [0.8, 1, 0.8], ...
    'BackgroundColor', [0.08, 0.08, 0.15], 'HorizontalAlignment', 'left');
estimated_time_display = uicontrol('Parent', flight_time_panel, 'Style', 'text', ...
    'String', '40.0 min', 'Position', [440, 38, 70, 18], ...
    'FontSize', 9, 'FontWeight', 'bold', 'ForegroundColor', [1, 1, 1], ...
    'BackgroundColor', [0.02, 0.02, 0.08], 'HorizontalAlignment', 'center', 'Tag', 'estimated_time');

% Remaining time
uicontrol('Parent', flight_time_panel, 'Style', 'text', ...
    'String', 'REMAINING:', 'Position', [340, 20, 100, 16], ...
    'FontSize', 8, 'FontWeight', 'bold', 'ForegroundColor', [0.8, 1, 0.8], ...
    'BackgroundColor', [0.08, 0.08, 0.15], 'HorizontalAlignment', 'left');
remaining_time_display = uicontrol('Parent', flight_time_panel, 'Style', 'text', ...
    'String', '40.0 min', 'Position', [440, 18, 70, 18], ...
    'FontSize', 9, 'FontWeight', 'bold', 'ForegroundColor', [0.3, 1, 0.8], ...
    'BackgroundColor', [0.02, 0.02, 0.08], 'HorizontalAlignment', 'center', 'Tag', 'remaining_time');

% ============ MISSION CONTROLS PANEL ============
control_panel = uipanel('Parent', dashboard, ...
    'Title', ' DRONE MISSION CONTROLS ', ...
    'FontSize', 11, 'FontWeight', 'bold', ...
    'BackgroundColor', [0.07, 0.07, 0.13], ...
    'ForegroundColor', [1, 0.8, 0.2], ...
    'BorderType', 'line', 'BorderWidth', 1.5, ...
    'Position', [0.41, 0.08, 0.57, 0.12]);

% START MISSION button - starts as green with "START MISSION"
start_btn = uicontrol('Parent', control_panel, 'Style', 'pushbutton', ...
    'String', 'START MISSION', 'Position', [220, 25, 150, 35], ...
    'FontSize', 12, 'FontWeight', 'bold', ...
    'BackgroundColor', [0, 0.7, 0.2], 'ForegroundColor', [1, 1, 1], ...
    'Callback', @startCallback, 'Tag', 'start_btn');

% RESET button
reset_btn = uicontrol('Parent', control_panel, 'Style', 'pushbutton', ... 
    'String', 'RESET', 'Position', [420, 25, 100, 35], ...
    'FontSize', 12, 'FontWeight', 'bold', ...
    'BackgroundColor', [0.5, 0.5, 0.5], 'ForegroundColor', [1, 1, 1], ...
    'Callback', @resetCallback, 'Tag', 'reset_btn');

% EXIT button
exit_btn = uicontrol('Parent', control_panel, 'Style', 'pushbutton', ... 
    'String', 'EXIT', 'Position', [570, 25, 100, 35], ...
    'FontSize', 12, 'FontWeight', 'bold', ...
    'BackgroundColor', [0.8, 0.2, 0.2], 'ForegroundColor', [1, 1, 1], ...
    'Callback', @exitCallback, 'Tag', 'exit_btn');

dashboard.UserData.handles.reset_btn = reset_btn;
dashboard.UserData.handles.exit_btn = exit_btn;

% GLOBAL STATUS BAR - starts with "READY - Press START to begin"
global_status = uicontrol('Parent', control_panel, 'Style', 'text', ...
    'String', 'READY - Press START to begin mission (SPACE = PAUSE)', ...
    'Position', [150, 5, 550, 18], ...
    'FontSize', 9, 'FontWeight', 'bold', ...
    'ForegroundColor', [0.8, 1, 0.8], 'BackgroundColor', [0.07, 0.07, 0.13], ...
    'HorizontalAlignment', 'center', 'Tag', 'global_status');

% ============ STORE HANDLES ============
dashboard.UserData.handles = struct();
dashboard.UserData.handles.lipo_voltage = lipo_voltage_display;
dashboard.UserData.handles.lipo_capacity = lipo_capacity_display;
dashboard.UserData.handles.lipo_soc = lipo_soc_display;
dashboard.UserData.handles.lipo_temp = lipo_temp_display;
dashboard.UserData.handles.lipo_bar = lipo_bar;
dashboard.UserData.handles.lipo_temp_warning = lipo_temp_warning;
dashboard.UserData.handles.lion_voltage = lion_voltage_display;
dashboard.UserData.handles.lion_capacity = lion_capacity_display;
dashboard.UserData.handles.lion_soc = lion_soc_display;
dashboard.UserData.handles.lion_temp = lion_temp_display;
dashboard.UserData.handles.lion_bar = lion_bar;
dashboard.UserData.handles.lion_temp_warning = lion_temp_warning;
dashboard.UserData.handles.current = current_display;
dashboard.UserData.handles.active = active_display;
dashboard.UserData.handles.voltage = voltage_display;
dashboard.UserData.handles.time = time_display;
dashboard.UserData.handles.status = status_text;
dashboard.UserData.handles.weather = weather_display;  
dashboard.UserData.handles.total_capacity = total_capacity_display;
dashboard.UserData.handles.remaining_capacity = remaining_capacity_display;
dashboard.UserData.handles.estimated_time = estimated_time_display;
dashboard.UserData.handles.remaining_time = remaining_time_display;
dashboard.UserData.handles.global_status = global_status;
dashboard.UserData.handles.drone_ax = drone_ax;
dashboard.UserData.handles.current_line = current_line;
dashboard.UserData.handles.voltage_ax = voltage_ax;
dashboard.UserData.handles.lipo_voltage_line = lipo_voltage_line;
dashboard.UserData.handles.lion_voltage_line = lion_voltage_line;
dashboard.UserData.handles.soc_ax = soc_ax;
dashboard.UserData.handles.lipo_soc_line = lipo_soc_line;
dashboard.UserData.handles.lion_soc_line = lion_soc_line;
dashboard.UserData.handles.total_soc_line = total_soc_line;
dashboard.UserData.handles.lipo_temp_line = lipo_temp_line;
dashboard.UserData.handles.lion_temp_line = lion_temp_line;
dashboard.UserData.handles.start_btn = start_btn;

fprintf('\nDASHBOARD DEPLOYED SUCCESSFULLY!\n');
fprintf('- Start button shows "START MISSION" initially\n');
fprintf('- Status shows "READY" until start is pressed\n');
fprintf('- Mission begins only after pressing START\n\n');

% ============ CALLBACK FUNCTIONS ============

    function startCallback(~, ~)
    if ~isvalid(dashboard)
        return;
    end
    
    % Toggle running state
    dashboard.UserData.running = ~dashboard.UserData.running;
    btn = dashboard.UserData.handles.start_btn;
    
    if dashboard.UserData.running
        % Mission started/resumed
        btn.String = 'PAUSE MISSION';
        btn.BackgroundColor = [0.8, 0.6, 0];
        dashboard.UserData.handles.status.String = 'MISSION IN PROGRESS';
        dashboard.UserData.handles.global_status.String = ...
            'MISSION IN PROGRESS - RETURN TO BASE AT 10% SOC - PRESS SPACE TO PAUSE';
        
        % Run the mission loop
        for idx = dashboard.UserData.i+1:dashboard.UserData.n_steps
            if ~isvalid(dashboard)
                return;
            end
            
            updateSimulation(dashboard, idx);
            dashboard.UserData.i = idx;  % ← CRITICAL: Update the current index
            pause(dashboard.UserData.dt);
            
            % Check if mission is complete
            if strcmp(dashboard.UserData.mission_phase, 'COMPLETE')
                dashboard.UserData.running = false;
                btn.String = 'START MISSION';
                btn.BackgroundColor = [0, 0.7, 0.2];
                dashboard.UserData.handles.status.String = 'MISSION COMPLETE - LANDED';
                dashboard.UserData.handles.global_status.String = ...
                    'MISSION COMPLETE - BATTERIES DEPLETED';
                break;
            end
            
            % Check for pause
            if ~dashboard.UserData.running
                dashboard.UserData.handles.status.String = 'MISSION PAUSED';
                dashboard.UserData.handles.global_status.String = ...
                    'MISSION PAUSED - Press RESUME to continue';
                break;
            end
        end
    else
        % Mission paused
        if ~strcmp(dashboard.UserData.mission_phase, 'COMPLETE')
            btn.String = 'RESUME MISSION';
            btn.BackgroundColor = [0, 0.7, 0.2];
            dashboard.UserData.handles.status.String = 'MISSION PAUSED';
            dashboard.UserData.handles.global_status.String = ...
                'MISSION PAUSED - Press RESUME to continue';
        end
    end
end

    function resetCallback(~, ~)
        if ~isvalid(dashboard)
            return;
        end
        
        % Reset all data to initial state
        dashboard.UserData.i = 1;
        dashboard.UserData.running = false;
        dashboard.UserData.lipo_discharged = 0;
        dashboard.UserData.lion_discharged = 0;
        dashboard.UserData.mission_phase = 'NORMAL';
        dashboard.UserData.return_start_index = 0;

        % Clear hysteresis and reserve flags
        if isfield(dashboard.UserData, 'last_selected')
                dashboard.UserData = rmfield(dashboard.UserData, 'last_selected');
        end
        if isfield(dashboard.UserData, 'lion_reserve_activated')
                dashboard.UserData.lion_reserve_activated = false;
        end
        if isfield(dashboard.UserData, 'lipo_reserve_mode')
                dashboard.UserData.lipo_reserve_mode = false;
        end
        if isfield(dashboard.UserData, 'lipo_reserve_capacity')  % ADD THIS
                dashboard.UserData = rmfield(dashboard.UserData, 'lipo_reserve_capacity');
        end

        % Reset battery values
        dashboard.UserData.lipo_voltage(1) = dashboard.UserData.lipo_voltage_full;
        dashboard.UserData.lion_voltage(1) = dashboard.UserData.lion_voltage_full;
        dashboard.UserData.lipo_capacity(1) = dashboard.UserData.lipo_capacity_Ah;
        dashboard.UserData.lion_capacity(1) = dashboard.UserData.lion_capacity_Ah;
        dashboard.UserData.lipo_temp(1) = 25;
        dashboard.UserData.lion_temp(1) = 25;
        dashboard.UserData.active_battery(1) = 0;

        % Reset drone position
        dashboard.UserData.drone_x(1) = dashboard.UserData.drone_x_pre(1);
        dashboard.UserData.drone_y(1) = dashboard.UserData.drone_y_pre(1);
        dashboard.UserData.drone_z(1) = dashboard.UserData.drone_z_pre(1);
        dashboard.UserData.drone_roll(1) = dashboard.UserData.drone_roll_pre(1);
        dashboard.UserData.drone_pitch(1) = dashboard.UserData.drone_pitch_pre(1);
        dashboard.UserData.drone_yaw(1) = dashboard.UserData.drone_yaw_pre(1);

        % Update displays
        updateDisplays(dashboard, 1);
        updatePlots(dashboard, 1);
        updateDrone(dashboard, 1);
        
        % Reset button and status
        btn = dashboard.UserData.handles.start_btn;
        btn.String = 'START MISSION';
        btn.BackgroundColor = [0, 0.7, 0.2];
        
        dashboard.UserData.handles.status.String = 'READY - PRESS START';
        dashboard.UserData.handles.global_status.String = ...
            'SYSTEM RESET - Press START to begin mission';
    end

    function exitCallback(~, ~)
        if isvalid(dashboard)
            delete(dashboard);
        end
    end

    function keyPressCallback(~, event)
        if ~isvalid(dashboard)
            return;
        end
        
        if strcmp(event.Key, 'space')
            % Toggle running state with spacebar
            dashboard.UserData.running = ~dashboard.UserData.running;
            btn = dashboard.UserData.handles.start_btn;
            
            if dashboard.UserData.running
                btn.String = 'PAUSE MISSION';
                btn.BackgroundColor = [0.8, 0.6, 0];
                dashboard.UserData.handles.status.String = 'MISSION IN PROGRESS';
                dashboard.UserData.handles.global_status.String = ...
                    'MISSION IN PROGRESS - PRESS SPACE TO PAUSE';
            else
                btn.String = 'RESUME MISSION';
                btn.BackgroundColor = [0, 0.7, 0.2];
                dashboard.UserData.handles.status.String = 'MISSION PAUSED';
                dashboard.UserData.handles.global_status.String = ...
                    'MISSION PAUSED - Press SPACE to resume';
            end
        end
    end

end

% ============ SIMULATION UPDATE FUNCTIONS ============
function updateSimulation(dashboard, i)
    if ~isvalid(dashboard)
        return;
    end
    
    % Handle first time step specially
    if i == 1
        % Just update displays and return
        updateDisplays(dashboard, 1);
        updatePlots(dashboard, 1);
        updateDrone(dashboard, 1);
        dashboard.UserData.i = 1;
        return;
    end
    
    dashboard.UserData.wind_speed = dashboard.UserData.wind_speed_history(i);
    
    % --- FIRST: Update drone position and load current for this time step ---
    if strcmp(dashboard.UserData.mission_phase, 'RETURN_TO_BASE')
        start_idx = dashboard.UserData.return_start_index;
        if start_idx > 0 && i > start_idx
            % Return to base logic
            current_x = dashboard.UserData.drone_x(i-1);
            current_y = dashboard.UserData.drone_y(i-1);
            current_z = dashboard.UserData.drone_z(i-1);
            
            target_x = dashboard.UserData.home_position(1);
            target_y = dashboard.UserData.home_position(2);
            target_z = 5;
            
            dashboard.UserData.drone_x(i) = current_x + (target_x - current_x) * 0.02;
            dashboard.UserData.drone_y(i) = current_y + (target_y - current_y) * 0.02;
            
            if current_z > target_z + 2
                dashboard.UserData.drone_z(i) = current_z - 0.5;
            else
                dashboard.UserData.drone_z(i) = max(0, current_z - 0.2);
            end
            
            % Use pre-generated current for return path or scale it
            if i <= length(dashboard.UserData.load_current_pre)
                dashboard.UserData.load_current(i) = max(3, dashboard.UserData.load_current_pre(i) * 0.95);
            else
                dashboard.UserData.load_current(i) = 5; % Default value
            end
            
            % Keep orientation from previous step during return
            dashboard.UserData.drone_roll(i) = dashboard.UserData.drone_roll(i-1);
            dashboard.UserData.drone_pitch(i) = dashboard.UserData.drone_pitch(i-1);
            dashboard.UserData.drone_yaw(i) = dashboard.UserData.drone_yaw(i-1);
        else
            % Before return path starts, use pre-generated data
            if i <= length(dashboard.UserData.drone_x_pre)
                dashboard.UserData.drone_x(i) = dashboard.UserData.drone_x_pre(i);
                dashboard.UserData.drone_y(i) = dashboard.UserData.drone_y_pre(i);
                dashboard.UserData.drone_z(i) = dashboard.UserData.drone_z_pre(i);
                dashboard.UserData.drone_roll(i) = dashboard.UserData.drone_roll_pre(i);
                dashboard.UserData.drone_pitch(i) = dashboard.UserData.drone_pitch_pre(i);
                dashboard.UserData.drone_yaw(i) = dashboard.UserData.drone_yaw_pre(i);
                dashboard.UserData.load_current(i) = dashboard.UserData.load_current_pre(i);
            end
        end
        
    elseif strcmp(dashboard.UserData.mission_phase, 'EMERGENCY_LANDING')
        dashboard.UserData.drone_z(i) = max(0, dashboard.UserData.drone_z(i-1) - 1.5);
        dashboard.UserData.load_current(i) = 2;
        % Keep x,y and orientation same as previous
        dashboard.UserData.drone_x(i) = dashboard.UserData.drone_x(i-1);
        dashboard.UserData.drone_y(i) = dashboard.UserData.drone_y(i-1);
        dashboard.UserData.drone_roll(i) = dashboard.UserData.drone_roll(i-1);
        dashboard.UserData.drone_pitch(i) = dashboard.UserData.drone_pitch(i-1);
        dashboard.UserData.drone_yaw(i) = dashboard.UserData.drone_yaw(i-1);
        
        if dashboard.UserData.drone_z(i) <= 0.05
            dashboard.UserData.running = false;
            dashboard.UserData.mission_phase = 'COMPLETE';
            if isvalid(dashboard)
                h = dashboard.UserData.handles;
                h.status.String = 'MISSION COMPLETE - LANDED';
                h.global_status.String = 'MISSION COMPLETE - BATTERIES DEPLETED';
            end
        end
        
    else  % NORMAL FLIGHT
        if i <= length(dashboard.UserData.drone_x_pre)
            dashboard.UserData.drone_x(i) = dashboard.UserData.drone_x_pre(i);
            dashboard.UserData.drone_y(i) = dashboard.UserData.drone_y_pre(i);
            dashboard.UserData.drone_z(i) = dashboard.UserData.drone_z_pre(i);
            dashboard.UserData.drone_roll(i) = dashboard.UserData.drone_roll_pre(i);
            dashboard.UserData.drone_pitch(i) = dashboard.UserData.drone_pitch_pre(i);
            dashboard.UserData.drone_yaw(i) = dashboard.UserData.drone_yaw_pre(i);
            dashboard.UserData.load_current(i) = dashboard.UserData.load_current_pre(i);
        end
    end
    
    % --- Now we have the correct load current for this time step ---
    I = dashboard.UserData.load_current(i);
    time_step = dashboard.UserData.dt / 3600;
    
    % ============ BATTERY SELECTION LOGIC - COMPLETE ============
% Calculate base voltages first
lipo_SoC = dashboard.UserData.lipo_capacity(i-1) / dashboard.UserData.lipo_capacity_Ah;
lion_SoC = dashboard.UserData.lion_capacity(i-1) / dashboard.UserData.lion_capacity_Ah;

dashboard.UserData.lipo_voltage(i) = 10.5 + 2.1*lipo_SoC + 0.2*sqrt(lipo_SoC) - 0.5*(1-lipo_SoC)^3;
dashboard.UserData.lion_voltage(i) = (3.0 + 1.2*lion_SoC + 0.15*log(1+9*lion_SoC)) * dashboard.UserData.lion_cells;

% Calculate current SOC for both batteries
current_lipo_soc = (dashboard.UserData.lipo_capacity(i-1) / dashboard.UserData.lipo_capacity_Ah) * 100;
current_lion_soc = (dashboard.UserData.lion_capacity(i-1) / dashboard.UserData.lion_capacity_Ah) * 100;

% Initialize flags if not exists
if ~isfield(dashboard.UserData, 'lipo_reserve_mode')
    dashboard.UserData.lipo_reserve_mode = false;
end
if ~isfield(dashboard.UserData, 'lion_reserve_activated')
    dashboard.UserData.lion_reserve_activated = false;
end
if ~isfield(dashboard.UserData, 'return_mode_activated')
    dashboard.UserData.return_mode_activated = false;
end

% Check if LiPo should enter reserve mode (≤10%) - SAVE FOR LANDING
if ~dashboard.UserData.lipo_reserve_mode && current_lipo_soc <= 10
    dashboard.UserData.lipo_reserve_mode = true;
    if isvalid(dashboard)
        h = dashboard.UserData.handles;
        h.status.String = 'LIPO RESERVE - SAVING FOR LANDING';
    end
    fprintf('LIPO RESERVE ACTIVATED at i=%d, SOC=%.1f%%\n', i, current_lipo_soc);
end

% Check if Li-ion is below reserve threshold (10%)
if ~dashboard.UserData.lion_reserve_activated && current_lion_soc <= 10
    dashboard.UserData.lion_reserve_activated = true;
    if isvalid(dashboard)
        h = dashboard.UserData.handles;
        h.status.String = 'LI-ION RESERVE - USING LIPO ONLY';
    end
end

% Check if we're in final approach for landing
time_to_landing = 2000 - dashboard.UserData.t(i);
final_approach = (time_to_landing > 0 && time_to_landing < 30);

% Check if we're in return to base mode
if strcmp(dashboard.UserData.mission_phase, 'RETURN_TO_BASE') && ~dashboard.UserData.return_mode_activated
    dashboard.UserData.return_mode_activated = true;
    if isvalid(dashboard)
        h = dashboard.UserData.handles;
        h.status.String = 'RETURN MODE - USING LI-ION, RESERVING LIPO';
    end
    fprintf('RETURN MODE ACTIVATED at i=%d\n', i);
end

% Check flight phases
time = dashboard.UserData.t(i);
altitude = dashboard.UserData.drone_z(i-1);
is_takeoff = time < 30;
is_rough_maneuver = (time >= 400 && time < 800) || (time >= 1200 && time < 1600);

% ============ LANDING DETECTION ============
is_descending = altitude < dashboard.UserData.drone_z(max(1,i-2)) + 0.1;
is_final_approach = altitude < 20 || ...
                   (strcmp(dashboard.UserData.mission_phase, 'RETURN_TO_BASE') && altitude < 30);
is_critical_landing = altitude < 5;
is_landing_phase = (altitude < 30 && is_descending) || ...
                   strcmp(dashboard.UserData.mission_phase, 'EMERGENCY_LANDING');

% Battery availability checks
lipo_has_capacity = dashboard.UserData.lipo_capacity(i-1) > 0.001;
lion_has_capacity = dashboard.UserData.lion_capacity(i-1) > 0.001;

% Voltage-based usability (for normal operation)
lipo_usable = lipo_has_capacity && ...
              dashboard.UserData.lipo_voltage(i) > dashboard.UserData.lipo_voltage_cutoff;
lion_usable = lion_has_capacity && ...
              dashboard.UserData.lion_voltage(i) > dashboard.UserData.lion_voltage_cutoff;

% High power availability (ignore voltage for emergency)
lipo_available_highpower = lipo_has_capacity;
lion_available_highpower = lion_has_capacity;

% ============ PRIORITY-BASED SELECTION ============

% PRIORITY 1: CRITICAL LANDING - MUST USE LIPO
if is_critical_landing && lipo_has_capacity
    selected = 1; % FORCE LiPo for final touchdown
    if isvalid(dashboard) && mod(i, 5) == 0
        h = dashboard.UserData.handles;
        h.status.String = 'FINAL DESCENT - USING LIPO';
    end

% PRIORITY 2: LANDING PHASE - USE RESERVED LIPO
elseif (is_landing_phase || is_final_approach) && lipo_has_capacity
    selected = 1; % Use LiPo for landing
    if isvalid(dashboard) && mod(i, 5) == 0
        h = dashboard.UserData.handles;
        h.status.String = 'LANDING PHASE - USING LIPO';
    end

% PRIORITY 3: RETURN TO BASE MODE - USE LI-ION IF USABLE, RESERVE LIPO
elseif strcmp(dashboard.UserData.mission_phase, 'RETURN_TO_BASE')
    if lion_usable && current_lion_soc > 3  % Use Li-ion if usable
        selected = 2; % Use Li-ion for return journey
        if isvalid(dashboard) && mod(i, 10) == 0
            h = dashboard.UserData.handles;
            h.status.String = 'RETURNING - USING LI-ION, LIPO RESERVED';
        end
    elseif lipo_usable
        selected = 1; % Emergency: use LiPo if Li-ion dead
        if isvalid(dashboard)
            h = dashboard.UserData.handles;
            h.status.String = 'RETURN EMERGENCY - USING LIPO';
        end
    elseif lipo_available_highpower  % Last resort: use LiPo even below cutoff
        selected = 1;
        if isvalid(dashboard)
            h = dashboard.UserData.handles;
            h.status.String = 'RETURN CRITICAL - LIPO BELOW CUTOFF';
        end
    else
        selected = 0;
    end

% PRIORITY 4: HIGH POWER DEMAND (>8.0A or critical phases)
elseif (I >= 8.0 || is_takeoff || is_rough_maneuver)
    % If LiPo is reserved AND we're not landing, use Li-ion if usable
    if dashboard.UserData.lipo_reserve_mode && lion_usable && current_lion_soc > 5
        selected = 2; % Use Li-ion to save LiPo
        if isvalid(dashboard) && mod(i, 5) == 0
            h = dashboard.UserData.handles;
            h.status.String = 'HIGH POWER - USING LI-ION, LIPO RESERVED';
        end
    elseif lipo_usable
        selected = 1; % Use LiPo for high power if usable
    elseif lipo_available_highpower
        selected = 1; % Use LiPo even below cutoff as last resort
    elseif lion_usable
        selected = 2; % Fallback to Li-ion if usable
    elseif lion_available_highpower
        selected = 2; % Fallback to Li-ion even below cutoff
    else
        selected = 0;
    end

% PRIORITY 5: LOW POWER CRUISE (I < 8.0A)
elseif I < 8.0
    % For low power, prefer Li-ion to save LiPo for landing
    if lion_usable && ~dashboard.UserData.lion_reserve_activated
        selected = 2; % Use Li-ion for cruising
        if isvalid(dashboard) && mod(i, 10) == 0 && dashboard.UserData.lipo_reserve_mode
            h = dashboard.UserData.handles;
            h.status.String = 'CRUISE - USING LI-ION, SAVING LIPO';
        end
    elseif lipo_usable
        selected = 1; % Use LiPo if Li-ion not usable
    elseif lipo_available_highpower
        selected = 1; % Use LiPo even below cutoff
    else
        selected = 0;
    end

% PRIORITY 6: FALLBACK
else
    if lipo_usable
        selected = 1;
    elseif lipo_available_highpower
        selected = 1;
    elseif lion_usable
        selected = 2;
    elseif lion_available_highpower
        selected = 2;
    else
        selected = 0;
    end
end

% Add simple hysteresis to prevent rapid switching (skip during landing)
if ~is_landing_phase && ~is_final_approach && isfield(dashboard.UserData, 'last_selected')
    if selected ~= dashboard.UserData.last_selected
        % Only switch if the difference is significant or current battery is dead
        current_battery_usable = (dashboard.UserData.last_selected == 1 && lipo_has_capacity) || ...
                                 (dashboard.UserData.last_selected == 2 && lion_has_capacity);
        
        if current_battery_usable && rand() > 0.3  % 30% chance to keep current battery
            selected = dashboard.UserData.last_selected;
        end
    end
end
dashboard.UserData.last_selected = selected;
dashboard.UserData.active_battery(i) = selected;

% Voltage sag - apply to active battery only
if selected == 1
    dashboard.UserData.lipo_voltage(i) = dashboard.UserData.lipo_voltage(i) - I * 0.025;
elseif selected == 2
    dashboard.UserData.lion_voltage(i) = dashboard.UserData.lion_voltage(i) - I * (0.06 * dashboard.UserData.lion_cells);
else
    dashboard.UserData.lipo_voltage(i) = dashboard.UserData.lipo_voltage(i) - I * 0.025;
    dashboard.UserData.lion_voltage(i) = dashboard.UserData.lion_voltage(i) - I * (0.06 * dashboard.UserData.lion_cells);
end

% Apply voltage limits
dashboard.UserData.lipo_voltage(i) = max(dashboard.UserData.lipo_voltage_cutoff, ...
    min(dashboard.UserData.lipo_voltage_full, dashboard.UserData.lipo_voltage(i)));
dashboard.UserData.lion_voltage(i) = max(dashboard.UserData.lion_voltage_cutoff, ...
    min(dashboard.UserData.lion_voltage_full, dashboard.UserData.lion_voltage(i)));

% ============ DISCHARGE LOGIC - SIMPLIFIED & FOOLPROOF ============
if selected == 1
    % Using LiPo
    if dashboard.UserData.lipo_reserve_mode
        % RESERVE MODE - Using reserved LiPo (should be during landing)
        % Need to actually discharge it!
        
        % Initialize reserve capacity if not done yet
        if ~isfield(dashboard.UserData, 'lipo_reserve_capacity')
            dashboard.UserData.lipo_reserve_capacity = dashboard.UserData.lipo_capacity(i-1);
        end
        
        % Discharge from the reserve capacity
        dashboard.UserData.lipo_reserve_capacity = max(0, ...
            dashboard.UserData.lipo_reserve_capacity - I * time_step);
        
        % Update the main capacity from reserve
        dashboard.UserData.lipo_capacity(i) = dashboard.UserData.lipo_reserve_capacity;
        
        % Update discharged tracker for consistency
        dashboard.UserData.lipo_discharged = dashboard.UserData.lipo_capacity_Ah - ...
            dashboard.UserData.lipo_reserve_capacity;
            
    else
        % NORMAL MODE - Regular LiPo discharge
        dashboard.UserData.lipo_discharged = dashboard.UserData.lipo_discharged + I * time_step;
        dashboard.UserData.lipo_capacity(i) = max(0, dashboard.UserData.lipo_capacity_Ah - ...
            dashboard.UserData.lipo_discharged);
    end
    dashboard.UserData.lion_capacity(i) = dashboard.UserData.lion_capacity(i-1);
    
elseif selected == 2
    % Using Li-ion
    if dashboard.UserData.lipo_reserve_mode
        % In reserve mode, LiPo uses stored value
        if ~isfield(dashboard.UserData, 'lipo_reserve_capacity')
            dashboard.UserData.lipo_reserve_capacity = dashboard.UserData.lipo_capacity(i-1);
        end
        dashboard.UserData.lipo_capacity(i) = dashboard.UserData.lipo_reserve_capacity;
    else
        % Normal mode - LiPo unchanged
        dashboard.UserData.lipo_capacity(i) = dashboard.UserData.lipo_capacity(i-1);
    end
    
    % Li-ion discharges normally
    dashboard.UserData.lion_discharged = dashboard.UserData.lion_discharged + I * time_step;
    dashboard.UserData.lion_capacity(i) = max(0, dashboard.UserData.lion_capacity_Ah - dashboard.UserData.lion_discharged);
    
else
    % DEPLETED mode
    if dashboard.UserData.lipo_capacity(i-1) > dashboard.UserData.lion_capacity(i-1)
        dashboard.UserData.lipo_discharged = dashboard.UserData.lipo_discharged + I * time_step;
        dashboard.UserData.lipo_capacity(i) = max(0, dashboard.UserData.lipo_capacity_Ah - dashboard.UserData.lipo_discharged);
        dashboard.UserData.lion_capacity(i) = dashboard.UserData.lion_capacity(i-1);
    else
        dashboard.UserData.lion_discharged = dashboard.UserData.lion_discharged + I * time_step;
        dashboard.UserData.lion_capacity(i) = max(0, dashboard.UserData.lion_capacity_Ah - dashboard.UserData.lion_discharged);
        dashboard.UserData.lipo_capacity(i) = dashboard.UserData.lipo_capacity(i-1);
    end
end

  % Temperature update
dashboard.UserData.lipo_temp(i) = dashboard.UserData.lipo_temp(i-1) + 0.015 * I - 0.006 * (dashboard.UserData.lipo_temp(i-1) - 25);
dashboard.UserData.lion_temp(i) = dashboard.UserData.lion_temp(i-1) + 0.008 * I - 0.006 * (dashboard.UserData.lion_temp(i-1) - 25);

% Calculate individual and combined SOC
lipo_soc = (dashboard.UserData.lipo_capacity(i) / dashboard.UserData.lipo_capacity_Ah) * 100; 
lion_soc = (dashboard.UserData.lion_capacity(i) / dashboard.UserData.lion_capacity_Ah) * 100; 
combined_soc = ((dashboard.UserData.lipo_capacity(i) + dashboard.UserData.lion_capacity(i)) / ...
                (dashboard.UserData.lipo_capacity_Ah + dashboard.UserData.lion_capacity_Ah)) * 100;

% Store 
dashboard.UserData.last_lipo_soc = lipo_soc;
dashboard.UserData.last_lion_soc = lion_soc;

% Smart return logic using LION SOC (reserve LiPo for landing)
if strcmp(dashboard.UserData.mission_phase, 'NORMAL')
    % Return when Li-ion hits 15% (LiPo reserved for landing)
    if current_lion_soc <= 15
        dashboard.UserData.mission_phase = 'RETURN_TO_BASE';
        dashboard.UserData.return_start_index = i;
        
        if isvalid(dashboard)
            h = dashboard.UserData.handles;
            h.status.String = 'LI-ION LOW - RETURNING (LIPO RESERVED)';
        end
        fprintf('RETURN TO BASE at i=%d: Li-ion=%.1f%%, LiPo=%.1f%% (reserved)\n', ...
            i, current_lion_soc, current_lipo_soc);
    end
    
elseif strcmp(dashboard.UserData.mission_phase, 'RETURN_TO_BASE')
    % During return, monitor both batteries
    % Emergency landing if:
    % 1. Combined SOC critically low (5%), OR
    % 2. Li-ion is depleted (≤1%) AND we're not on final approach
    if combined_soc <= 5 || (current_lion_soc <= 1 && ~final_approach)
        dashboard.UserData.mission_phase = 'EMERGENCY_LANDING';
        
        if isvalid(dashboard)
            h = dashboard.UserData.handles;
            h.status.String = 'EMERGENCY LANDING';
        end
    end
end

% Check if both batteries are at zero
if dashboard.UserData.lipo_capacity(i) <= 0.001 && dashboard.UserData.lion_capacity(i) <= 0.001
    if i < dashboard.UserData.n_steps
        dashboard.UserData.mission_phase = 'EMERGENCY_LANDING';
    end
end

% Update displays and plots
updateDisplays(dashboard, i);
updatePlots(dashboard, i);
updateDrone(dashboard, i);

dashboard.UserData.i = i;
end

function updateDisplays(dashboard, i)
    if ~isvalid(dashboard)
        return;
    end
    
    h = dashboard.UserData.handles;

    % Constants - defined ONCE at the top
    max_bar_chars = 100;  % For progress bars (0-100% maps to 0-100 chars)
    
    % LiPo displays
    h.lipo_voltage.String = sprintf('%.2f V', dashboard.UserData.lipo_voltage(i));
    h.lipo_capacity.String = sprintf('%.0f mAh', dashboard.UserData.lipo_capacity(i)*1000);
    lipo_soc = dashboard.UserData.lipo_capacity(i) / dashboard.UserData.lipo_capacity_Ah * 100;
    h.lipo_soc.String = sprintf('%.1f %%', lipo_soc);
    h.lipo_temp.String = sprintf('%.1f C', dashboard.UserData.lipo_temp(i));
    
    % LiPo temperature warnings - using temp thresholds
    if dashboard.UserData.lipo_temp(i) >= dashboard.UserData.lipo_temp_critical
        h.lipo_temp_warning.String = 'CRITICAL';
        h.lipo_temp_warning.ForegroundColor = [1, 0, 0];
        h.lipo_temp_warning.FontSize = 10;
        h.lipo_temp_warning.FontWeight = 'bold';
    elseif dashboard.UserData.lipo_temp(i) >= dashboard.UserData.lipo_temp_max
        h.lipo_temp_warning.String = 'WARNING';
        h.lipo_temp_warning.ForegroundColor = [1, 0.5, 0];
        h.lipo_temp_warning.FontSize = 10;
        h.lipo_temp_warning.FontWeight = 'bold';
    else
        h.lipo_temp_warning.String = '';
    end
    
    % LiPo progress bar
    bar_length = round(lipo_soc / 100 * max_bar_chars);
    if bar_length > 0
        h.lipo_bar.String = repmat('|', 1, bar_length);
        % Add color based on SOC
        if lipo_soc < 20
            h.lipo_bar.ForegroundColor = [1, 0, 0];  % Red for low
        elseif lipo_soc < 40
            h.lipo_bar.ForegroundColor = [1, 0.5, 0];  % Orange for warning
        else
            h.lipo_bar.ForegroundColor = [0, 1, 0.6];  % Green for normal
        end
    else
        h.lipo_bar.String = '';
    end
    
    % Li-ion displays
    h.lion_voltage.String = sprintf('%.2f V', dashboard.UserData.lion_voltage(i));
    h.lion_capacity.String = sprintf('%.0f mAh', dashboard.UserData.lion_capacity(i)*1000);
    lion_soc = dashboard.UserData.lion_capacity(i) / dashboard.UserData.lion_capacity_Ah * 100;
    h.lion_soc.String = sprintf('%.1f %%', lion_soc);
    h.lion_temp.String = sprintf('%.1f C', dashboard.UserData.lion_temp(i));
    
    % Li-ion temperature warnings - using temp thresholds
    if dashboard.UserData.lion_temp(i) >= dashboard.UserData.lion_temp_critical
        h.lion_temp_warning.String = 'CRITICAL';
        h.lion_temp_warning.ForegroundColor = [1, 0, 0];
        h.lion_temp_warning.FontSize = 10;
        h.lion_temp_warning.FontWeight = 'bold';
    elseif dashboard.UserData.lion_temp(i) >= dashboard.UserData.lion_temp_max
        h.lion_temp_warning.String = 'WARNING';
        h.lion_temp_warning.ForegroundColor = [1, 0.5, 0];
        h.lion_temp_warning.FontSize = 10;
        h.lion_temp_warning.FontWeight = 'bold';
    else
        h.lion_temp_warning.String = '';
    end
    
    % Li-ion progress bar
    bar_length = round(lion_soc / 100 * max_bar_chars);
    if bar_length > 0
        h.lion_bar.String = repmat('|', 1, bar_length);
        % Add color based on SOC
        if lion_soc < 20
            h.lion_bar.ForegroundColor = [1, 0, 0];  % Red for low
        elseif lion_soc < 40
            h.lion_bar.ForegroundColor = [1, 0.5, 0];  % Orange for warning
        else
            h.lion_bar.ForegroundColor = [1, 0.4, 0.7];  % Pink for normal
        end
    else
        h.lion_bar.String = '';
    end
    
    % Flight instruments
    h.current.String = sprintf('%.1f A', dashboard.UserData.load_current(i));
    
    if dashboard.UserData.active_battery(i) == 1
    h.active.String = 'LIPO';
    h.active.ForegroundColor = [0.3, 1, 0.8];
    h.voltage.String = sprintf('%.2f V', dashboard.UserData.lipo_voltage(i));
elseif dashboard.UserData.active_battery(i) == 2
    h.active.String = 'LI-ION';
    h.active.ForegroundColor = [1, 0.6, 0.8];
    h.voltage.String = sprintf('%.2f V', dashboard.UserData.lion_voltage(i));
else
    % DEPLETED mode - show which battery is still being used
    if dashboard.UserData.lipo_capacity(i) > dashboard.UserData.lion_capacity(i)
        h.active.String = 'LIPO (LOW)';
        h.voltage.String = sprintf('%.2f V', dashboard.UserData.lipo_voltage(i));
    else
        h.active.String = 'LI-ION (LOW)';
        h.voltage.String = sprintf('%.2f V', dashboard.UserData.lion_voltage(i));
    end
    h.active.ForegroundColor = [1, 0.5, 0]; % Orange for low battery
    end
    
    minutes = floor(dashboard.UserData.t(i)/60);
    seconds = mod(floor(dashboard.UserData.t(i)), 60);
    h.time.String = sprintf('%d:%02d', minutes, seconds);

    % WEATHER INDICATOR UPDATE
if isfield(h, 'weather') && isfield(dashboard.UserData, 'wind_speed')
    current_wind = dashboard.UserData.wind_speed;
    
    if current_wind > 5
        h.weather.String = 'STORMY';
        h.weather.ForegroundColor = [1, 0, 0];
    elseif current_wind > 3
        h.weather.String = 'WINDY';
        h.weather.ForegroundColor = [1, 0.5, 0];
    elseif current_wind > 1
        h.weather.String = 'BREEZY';
        h.weather.ForegroundColor = [1, 1, 0];
    else
        h.weather.String = 'CALM';
        h.weather.ForegroundColor = [0.5, 1, 0.5];
    end
end
    
    % Update status based on mission phase
    if strcmp(dashboard.UserData.mission_phase, 'RETURN_TO_BASE')
        h.status.String = sprintf('RETURNING TO BASE - SOC: %.1f%%', ...
            (dashboard.UserData.lipo_capacity(i) + dashboard.UserData.lion_capacity(i)) / ...
            (dashboard.UserData.lipo_capacity_Ah + dashboard.UserData.lion_capacity_Ah) * 100);
    elseif strcmp(dashboard.UserData.mission_phase, 'EMERGENCY_LANDING')
        h.status.String = 'BATTERIES DEPLETED - EMERGENCY LANDING';
    elseif ~dashboard.UserData.running && i == 1
        h.status.String = 'READY - PRESS START';
    elseif ~dashboard.UserData.running
        h.status.String = 'MISSION PAUSED';
    else
        h.status.String = sprintf('FLIGHT TIME: %d:%02d ACTIVE: %s', ...
            minutes, seconds, h.active.String);
    end
    
  % Capacity and flight time estimation
total_cap = (dashboard.UserData.lipo_capacity_Ah + dashboard.UserData.lion_capacity_Ah) * 1000;
remaining_cap = (dashboard.UserData.lipo_capacity(i) + dashboard.UserData.lion_capacity(i)) * 1000;
used_cap = total_cap - remaining_cap;

h.total_capacity.String = sprintf('%.0f mAh', total_cap);
h.remaining_capacity.String = sprintf('%.0f mAh', remaining_cap);

if used_cap > 0 && i > 10  % Wait for at least 10 samples for stability
    elapsed_min = dashboard.UserData.t(i)/60;
    
    % Use moving average of last 60 seconds for more stable estimation
    window_size = min(60, i-1);  % Last 60 samples or all available
    recent_current = mean(dashboard.UserData.load_current(i-window_size+1:i));
    
    % Calculate average voltage (for power estimation)
    avg_voltage = (dashboard.UserData.lipo_voltage(i) + dashboard.UserData.lion_voltage(i)) / 2;
    
    % Instantaneous consumption rate based on recent current
    instant_consumption_rate = recent_current * avg_voltage / 1000 * 60;  % mAh/min
    
    % Overall consumption rate since start
    overall_consumption_rate = used_cap / elapsed_min;  % mAh/min
    
    % Blend short-term and long-term estimates (70% long-term, 30% short-term)
    % This prevents wild fluctuations while still responding to changing conditions
    blended_rate = 0.7 * overall_consumption_rate + 0.3 * instant_consumption_rate;
    
    if blended_rate > 0
        remaining_min = remaining_cap / blended_rate;
        total_est_min = elapsed_min + remaining_min;
        
        % Add sanity checks to prevent unrealistic estimates
        if remaining_min > 60
            remaining_min = 60;  % Cap at 60 minutes max
        end
        if remaining_min < 0.5
            remaining_min = 0.5;  % Minimum 30 seconds
        end
        
        h.estimated_time.String = sprintf('%.1f min', total_est_min);
        h.remaining_time.String = sprintf('%.1f min', remaining_min);
    end
elseif i <= 10
    % Initial flight - use default estimate
    h.estimated_time.String = '--.- min';
    h.remaining_time.String = '--.- min';
end
    
    if isfield(h, 'global_status') && isvalid(h.global_status)
        total_soc = (dashboard.UserData.lipo_capacity(i) + dashboard.UserData.lion_capacity(i)) / ...
            (dashboard.UserData.lipo_capacity_Ah + dashboard.UserData.lion_capacity_Ah) * 100;
        
        if strcmp(dashboard.UserData.mission_phase, 'RETURN_TO_BASE')
            h.global_status.String = sprintf('RETURNING TO BASE - SOC: %.1f%% - DISTANCE: %.0fm', ...
                total_soc, sqrt(dashboard.UserData.drone_x(i)^2 + dashboard.UserData.drone_y(i)^2));
        elseif strcmp(dashboard.UserData.mission_phase, 'EMERGENCY_LANDING')
            h.global_status.String = 'EMERGENCY LANDING - BATTERIES DEPLETED';
        elseif ~dashboard.UserData.running && i == 1
            h.global_status.String = 'READY - Press START to begin mission (SPACE = PAUSE)';
        elseif ~dashboard.UserData.running
            h.global_status.String = 'MISSION PAUSED - Press RESUME to continue';
        else
            h.global_status.String = sprintf('MISSION %d:%02d - SOC: %.1f%% - ACTIVE: %s - SPACE = PAUSE', ...
                minutes, seconds, total_soc, h.active.String);
        end
    end
end

function updatePlots(dashboard, i)
    if ~isvalid(dashboard)
        return;
    end
    
    h = dashboard.UserData.handles;
    t_current = dashboard.UserData.t(1:i);
    
    set(h.current_line, 'XData', t_current, 'YData', dashboard.UserData.load_current(1:i));
    xlim(h.current_line.Parent, [0, max(dashboard.UserData.t(end), dashboard.UserData.t(i)+10)]);
    
    set(h.lipo_voltage_line, 'XData', t_current, 'YData', dashboard.UserData.lipo_voltage(1:i));
    set(h.lion_voltage_line, 'XData', t_current, 'YData', dashboard.UserData.lion_voltage(1:i));
    xlim(h.lipo_voltage_line.Parent, [0, max(dashboard.UserData.t(end), dashboard.UserData.t(i)+10)]);
    
    lipo_soc = dashboard.UserData.lipo_capacity(1:i) / dashboard.UserData.lipo_capacity_Ah * 100;
    lion_soc = dashboard.UserData.lion_capacity(1:i) / dashboard.UserData.lion_capacity_Ah * 100;
    total_soc = (dashboard.UserData.lipo_capacity(1:i) + dashboard.UserData.lion_capacity(1:i)) / ...
        (dashboard.UserData.lipo_capacity_Ah + dashboard.UserData.lion_capacity_Ah) * 100;
    
    set(h.lipo_soc_line, 'XData', t_current, 'YData', lipo_soc);
    set(h.lion_soc_line, 'XData', t_current, 'YData', lion_soc);
    set(h.total_soc_line, 'XData', t_current, 'YData', total_soc);
    xlim(h.lipo_soc_line.Parent, [0, max(dashboard.UserData.t(end), dashboard.UserData.t(i)+10)]);
    
    set(h.lipo_temp_line, 'XData', t_current, 'YData', dashboard.UserData.lipo_temp(1:i));
    set(h.lion_temp_line, 'XData', t_current, 'YData', dashboard.UserData.lion_temp(1:i));
    xlim(h.lipo_temp_line.Parent, [0, max(dashboard.UserData.t(end), dashboard.UserData.t(i)+10)]);
    
    drawnow limitrate;
end

function updateDrone(dashboard, i)
    if ~isvalid(dashboard)
        return;
    end
    
    h = dashboard.UserData.handles;
    ax = h.drone_ax;
    if ~isvalid(ax)
        return;
    end
    
    drone = ax.UserData;
    
    x = dashboard.UserData.drone_x(i);
    y = dashboard.UserData.drone_y(i);
    z = max(0, dashboard.UserData.drone_z(i));
    
    roll = dashboard.UserData.drone_roll(i)*pi/180;
    pitch = dashboard.UserData.drone_pitch(i)*pi/180;
    yaw = dashboard.UserData.drone_yaw(i)*pi/180;
    
    Rx = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
    Ry = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
    Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
    R = Rz * Ry * Rx;
    
    [Xb, Yb, Zb] = cube(0, 0, 0, 2);
    for k = 1:numel(Xb)
        p = R * [Xb(k); Yb(k); Zb(k)];
        Xb(k) = p(1) + x;
        Yb(k) = p(2) + y;
        Zb(k) = p(3) + z;
    end
    if isvalid(drone.body)
        set(drone.body, 'XData', Xb, 'YData', Yb, 'ZData', Zb);
    end
    
    arm_length = 4;
    arm_points = [-arm_length 0 0; arm_length 0 0; 0 -arm_length 0; 0 arm_length 0];
    
    for k = 1:4
        if isvalid(drone.arms(k)) && isvalid(drone.props(k))
            p1 = R * arm_points(k,:)';
            p2 = R * (arm_points(k,:) * 0.3)';
            p1 = p1 + [x; y; z];
            p2 = p2 + [x; y; z];
            
            set(drone.arms(k), 'XData', [p1(1) p2(1)], 'YData', [p1(2) p2(2)], 'ZData', [p1(3) p2(3)]);
            set(drone.props(k), 'XData', p1(1), 'YData', p1(2), 'ZData', p1(3));
        end
    end
    
    trail_len = 80;
    start_idx = max(1, i-trail_len);
    xs = dashboard.UserData.drone_x(start_idx:i);
    ys = dashboard.UserData.drone_y(start_idx:i);
    zs = dashboard.UserData.drone_z(start_idx:i);
    if isvalid(drone.trail)
        set(drone.trail, 'XData', xs, 'YData', ys, 'ZData', zs);
    end
    
    margin = 25;
    xlim(ax, [x-margin x+margin]);
    ylim(ax, [y-margin y+margin]);
    zlim(ax, [0 max(40, z+20)]);
    drawnow limitrate;
end

function [X, Y, Z] = cube(xc, yc, zc, s)
    v = [-1, -1, -1; 1, -1, -1; 1, 1, -1; -1, 1, -1; 
         -1, -1, 1; 1, -1, 1; 1, 1, 1; -1, 1, 1] * s/2;
    v = v + [xc, yc, zc];
    f = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    [X, Y, Z] = deal(zeros(4, size(f,1)));
    for idx = 1:size(f,1)
        X(:,idx) = v(f(idx,:),1);
        Y(:,idx) = v(f(idx,:),2);
        Z(:,idx) = v(f(idx,:),3);
    end
end

function drone = createDrone(ax)
    [Xb, Yb, Zb] = cube(0, 0, 0, 2);
    drone.body = patch(ax, Xb, Yb, Zb, [0.2 0.4 0.8],...
        'FaceAlpha', 0.8, 'EdgeColor', [0 1 1], 'LineWidth', 1);
    
    drone.arms = gobjects(4,1);
    for k = 1:4
        drone.arms(k) = plot3(ax, [0 0], [0 0], [0 0], 'w-', 'LineWidth', 2);
    end
    
    colors = {[0 1 0], [1 0 0], [0 1 0], [1 0 0]};
    drone.props = gobjects(4,1);
    for k = 1:4
        drone.props(k) = scatter3(ax, 0, 0, 0, 80, colors{k}, 'filled');
    end
    
    drone.trail = plot3(ax, 0, 0, 0, 'c-', 'LineWidth', 1, 'Color', [0 1 1 0.3]);
    
    ax.UserData = drone;
end
