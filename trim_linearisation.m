% Script for trim and linearization of chapter 5
% * equation 5.6 included in LIN_F16BLOCK
% * For the default flight condition and low fidelity model the following
% calculations are done...

% Script is largely derived from findF16Dynamics.m created by Ewoud Smeur
clc;
clear variables;
close all;

% fidelity flag, 1 is high fidelity, 0 is low fidelity
global fi_flag_Simulink
global x_a
% disp('1.  Steady Wings-Level Flight.');
% disp('2.  Steady Turning Flight.');
% disp('3.  Steady Pull-Up Flight.');
% disp('4.  Steady Roll Flight.');
FC_flag = 1; 
ft2m = 0.3048;

%% Input parameters
%
altitude = 15000; % Altitude in ft
velocity = 500; % Velocity in ft/s

% Initial guess for trim
%
thrust = 5000;          % thrust, lbs
elevator = -0.09;       % elevator, degrees
alpha = 8.49;              % AOA, degrees
rudder = -0.01;             % rudder angle, degrees
aileron = 0.01;            % aileron, degrees


%% Parameters for linear model
dT = 0.001;
T_start = 0;
T_end = 0.2;
% Create a linearly spaced vector consisting of time points 
time_array = linspace(0,T_end,T_end/dT + 1);

neg_step = -ones(size(time_array));
thrust = 0;
% Accelerometer positions in feet
accel_pos = [0.0, 5.0, 5.8, 5.9, 6.0, 7.0, 15.0];
%linestyles = ["-","--","-.",".",":",".."];
figure('Renderer', 'painters', 'Position', [10 10 800 600]);
%% Iterate over accelerometer positions
for index = 1:7 %numel(accel_pos)
    x_a = accel_pos(index)*ft2m
    % Q 5.2
    fi_flag_Simulink = 0;
    [trim_state_lo, trim_thrust_lo, trim_control_lo, dLEF, xu_lo] = trim_F16(thrust, elevator, alpha, aileron, rudder, velocity, altitude, FC_flag);
    
    %% Find the state space model for the lofi model at the desired alt and vel.
    %%
    trim_state_lin = trim_state_lo; trim_thrust_lin = trim_thrust_lo; trim_control_lin = trim_control_lo;
    operating_point = operpoint('LIN_F16Block'); % retrieves initial conditions from integrators
    operating_point.Inputs(1).u = trim_thrust_lin; operating_point.Inputs(2).u = trim_control_lin(1);
    operating_point.Inputs(3).u = trim_control_lin(2); operating_point.Inputs(4).u = trim_control_lin(3);
    
    SS_lo = linearize('LIN_F16Block');

    tf_system = tf(SS_lo);

    % minimum realization or pole-zero cancellation at the system 
    % for the elevator to normal acceleration
    tf_an_el = minreal(tf_system(19,2)); % 19 is the row, 2 is the column corresponds to normal acceleration and elevator input
    
    zero_tf_an_el = zero(tf_an_el);

    if index == 1
        % Q 5.3
        cg_sslo = SS_lo
%         cg_C = SS_lo.C
%         writematrix(SS_lo.C, 'xa0_ssC.txt')
%         writematrix(SS_lo.D, 'xa0_ssD.txt')
        % Q 5.5
        x_a
        disp("Q5.5 elevator-to-normal-acceleration transfer function")
        tf_an_el
        tfcg = tf_an_el
    end
    % Q 5.9
    disp("Q5.9 Zeros")
    zero_tf_an_el

    % Q 5.6
    disp("Q5.6 Normal acceleration response")
    [y, t, u] = lsim(tf_an_el, neg_step, time_array);
    plot(t, y); %,linestyles(index)
    hold on
end
hold off
grid on
% xlim([0 1])
fontsize(gcf,scale=1.8)
legend('x_a = ' + string(accel_pos(1)) +' ft','x_a = ' + string(accel_pos(2)) +' ft', 'x_a = ' + string(accel_pos(3)) +' ft', 'x_a = ' + string(accel_pos(4)) +' ft', 'x_a = ' + string(accel_pos(5)) +' ft', 'x_a = ' + string(accel_pos(6)) +' ft', 'x_a = ' + string(accel_pos(7)) +' ft','Location','southeast')
% title('Negative step response results for normal acceleration at various accelerometer positions')
xlabel('Time [s]') 
ylabel('Normal Acceleration [m/s^2]') 
% saveas(gcf,"ch5_normal_accelerations.png")
ax = gcf;
% Requires R2020a or later
exportgraphics(ax,"ch5_normal_accelerations.png",'Resolution',300) 
xlim([0 0.004])
ax = gcf;
% Requires R2020a or later
exportgraphics(ax,"ch5_normal_accelerations_2.png",'Resolution',300) 