%% Chapter 6 and 7 answers
clc;
clear variables;
close all;
%% chapter 5 but with selected condition of lastname student number
global fi_flag_Simulink
global x_a
x_a = 0;
FC_flag = 1; 
fi_flag_Simulink = 0; % low fidelity model chosen
altitude = 10000; % Altitude in ft
ft2m = 0.3048;

velocity = 350; % Velocity in ft/s
% velocity = 475; % Velocity in ft/s middle
% velocity = 600; % Velocity in ft/s neighbour

% velocity = 443; % Velocity in ft/s operating range

gravity = 9.80665;

thrust = 5000;          % thrust, lbs
elevator = -0.09;       % elevator, degrees
alpha = 8.49;              % AOA, degrees
rudder = -0.01;             % rudder angle, degrees
aileron = 0.01;            % aileron, degrees
%We take the SS_lo from FindF16Dynamics then reduce it accordingly
[trim_state_lo, trim_thrust_lo, trim_control_lo, dLEF, xu_lo] = trim_F16(thrust, elevator, alpha, aileron, rudder, velocity, altitude, FC_flag);
trim_state_lin = trim_state_lo; trim_thrust_lin = trim_thrust_lo; trim_control_lin = trim_control_lo;
operating_point = operpoint('LIN_F16Block'); % retrieves initial conditions from integrators
operating_point.Inputs(1).u = trim_thrust_lin; operating_point.Inputs(2).u = trim_control_lin(1);
operating_point.Inputs(3).u = trim_control_lin(2); operating_point.Inputs(4).u = trim_control_lin(3);
SS_lo = linearize('LIN_F16Block');
% [A_lo,B_lo,C_lo,D_lo] = linmod('LIN_F16Block', [trim_state_lin; trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3); dLEF; -trim_state_lin(8)*180/pi], [trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3)]);


%% chapter 6 answers
%% Longitudinal reduction and short period version


% A_long = SS_lo.A([5 7 8 11], [5 7 8 11]);

% Here we select the V, alpha, theta, q 
A_long = SS_lo.A([7 8 5 11], [7 8 5 11]);
A_long_sp = [A_long([2 4],[2 4])];
SS_lo.A([8 11 14], [8 11 14]) 
% show that the short period A matrix is similar to the one given in the assignment
% A_long = SS_lo.A([8 11], [8 11]);

% Here we select the elevator input
% B_long = SS_lo.A([5 7 8 11], 14); 
B_long = SS_lo.A([7 8 5 11], 14);
B_long_sp = [B_long([2 4])];
[SS_lo.B([8 11 14],2)]

% Reduced C Matrix
% C_long = SS_lo.C([5 7 8 11], [5 7 8 11]);
C_long = SS_lo.C([7 8 5 11], [7 8 5 11]);
C_long_sp = [C_long([2 4],[2 4])];
% Reduced D Matrix
% D_long = SS_lo.C([5 7 8 11], 14);
D_long = SS_lo.C([7 8 5 11], 14);
D_long_sp  = D_long([2 4],1);

% Make SS system out of these matrices

long_ss = ss(A_long, B_long, C_long, D_long, ...
                    'StateName', {'V', 'alpha', 'theta', 'q'}, ... 
                     'InputName', {'delta_e'}, ...
                     'OutputName', {'V', 'alpha', 'theta', 'q'});
eig_long = eig(long_ss);


%% Lateral reduction
% Here we select beta, phi, p and r from A matrix
A_lat = SS_lo.A([9 4 10 12], [9 4 10 12]);

% Here we select the aileron and rudder input
B_lat = SS_lo.A([9 4 10 12], [15 16]);

% Reduced C Matrix
C_lat = SS_lo.C([9 4 10 12], [9 4 10 12]);

% Reduced D Matrix
D_lat = SS_lo.C([9 4 10 12], [15 16]);

% Make SS system out of these matrices

lat_ss = ss(A_lat, B_lat, C_lat, D_lat);
eig_lat = eig(lat_ss);
%pzmap(lat_ss)

%% Short period
% Calculates natural frequency and damping coefficients for
% longitudonal system
[wn_long, zeta_long] = damp(long_ss);

sp_eig1 = eig_long(1);
sp_eig2 = eig_long(2);

%Finding wn, zeta, period and time to half amplitude
sp_wn = wn_long(3);
sp_zeta = zeta_long(3);
sp_period = 2*pi/imag(sp_eig1);
sp_T12 = log(0.5)/real(sp_eig1);

%% Phugoid
phu_eig1 = eig_long(3);
phu_eig2 = eig_long(4);

%Finding wn, zeta, period and time to half amplitude
phu_wn = wn_long(1);
phu_zeta = zeta_long(1);
phu_period = 2*pi/imag(phu_eig1);
phu_T12 = log(0.5)/real(phu_eig1);

%% Dutch roll
%Calculates natural frequencies and damping coefficients for lateral system
[wn_lat, zeta_lat] = damp(lat_ss);

dr_eig1 = eig_lat(1);
dr_eig2 = eig_lat(2);

%Finding wn, zeta, period and time to half amplitude
dr_wn = wn_lat(3);
dr_zeta = zeta_lat(3);
dr_period = 2*pi/imag(dr_eig1);
dr_T12 = log(0.5)/real(dr_eig1);

%% Aperiodic roll
ar_eig = eig_lat(3);

%Finding wn, time constant and time to half amplitude
ar_wn = wn_lat(2);
ar_t = -1/ar_eig;
ar_T12 = log(0.5)/real(ar_eig);

%% Spiral
spi_eig = eig_lat(4);

%Finding wn, time constant and time to half amplitude
spi_wn = wn_lat(1);
spi_t = -1/spi_eig;
spi_T12 = log(0.5)/real(spi_eig);


%% Chapter 7 steps 1, 2, and 3

%% step 7.1
% Create a short period model from the model without actuator dynamics,
% just calculated in chapter 6. This short period model only involves two
% states: angle of attack and pitch rate

long_sp_ss = ss(A_long_sp, B_long_sp, C_long_sp, D_long_sp, ...
                    'StateName', {'alpha', 'q'}, ... 
                     'InputName', {'delta_e'}, ...
                     'OutputName', {'alpha', 'q'});

% make a transfer function from the systems and 
% then minreal the transfer function which does a minimal realization 
% and a pole-zero cancellation
tf_q_de       = minreal(tf(long_sp_ss('q')));
tf_alpha_de   = minreal(tf(long_sp_ss('alpha')));

% calculate the 2 state model's natural frequency and damping of linear
% system dynamics
[w_n_sp,damp_ratio_sp] = damp(long_sp_ss);
% poles
w_n_sp_sys    = w_n_sp(1); 
damp_ratio_sp_sys  = damp_ratio_sp(1); 
% zeros
tf_num = cell2mat(tf_q_de.num);
T_theta_2_sys = tf_num(2)/tf_num(3); % used later for the lead lag filter

%% step 7.2
% compare the step input time responses of pitch rate q for the 4 state vs
% reduced 2 state model both without actuator dynamics. Analyze these time
% response of q and determine which is the dominant mode
timestep = 0.005;

t_arr = 0:timestep:15;
step_4state = step(-long_ss, t_arr);
step_2state = step(-long_sp_ss, t_arr);

figure('Renderer', 'painters', 'Position', [10 10 800 600]);
plot(t_arr, step_4state(:,4), '-', 'LineWidth',2);
hold on;
plot(t_arr, step_2state(:,2), ':', 'LineWidth',2);
grid on;
hold on;
legend("4 state model w/o actuator dynamics", "reduced 2 state model w/o actuator dynamics",'location','southeast');
% title("Step input response of pitch rate");
xlabel("Time [s]")
ylabel("Pitch Rate [deg/s]");
fontsize(gcf,scale=1.8)
ax = gcf;
% Requires R2020a or later
exportgraphics(ax,sprintf("figures/ch7_1_step_q_%0.f.png",velocity),'Resolution',300) 

%
timestep = 0.005;

t_arr = 0:timestep:250;
step_4state = step(-long_ss, t_arr);
step_2state = step(-long_sp_ss, t_arr);

figure('Renderer', 'painters', 'Position', [10 10 800 600]);
plot(t_arr, step_4state(:,4), '-', 'LineWidth',2);
hold on;
plot(t_arr, step_2state(:,2), ':', 'LineWidth',2);
grid on;
hold on;
legend("4 state model w/o actuator dynamics", "reduced 2 state model w/o actuator dynamics",'location','southeast');
% title("Step input response of pitch rate");
xlabel("Time [s]")
ylabel("Pitch Rate [deg/s]");
fontsize(gcf,scale=1.8)
ax = gcf;
% Requires R2020a or later
exportgraphics(ax,sprintf("figures/ch7_1_step_q_250s_%0.f.png",velocity),'Resolution',300) 


%% Chapter 7 steps 3, 4, 5, 6, 7 8

%% step 3 requirements of CAP and Gibson criteria converted to frequency domain
% natural frequency w_n_sp, damping ratio damp_ratio, and time constant

req_w_n_sp = 0.03 * velocity*ft2m;
req_T_theta_2 = 1/(0.75*req_w_n_sp);
req_damp_ratio = 0.5;

%% step 4 
% Calculate the gains needed to meet the step 3 requirements
% use that to find a pitch rate controller
% check the gusts
% 
% from the requirements let's calculate the required poles
% and see what the gust feedback is
% pole placement to find appropriate feedback gain

% root locus rltool(tf_q_de) alternatively to find a damped system 
% or poles can be found mathematically as it is a second order transfer
% function
a = -req_damp_ratio*req_w_n_sp;
b = req_w_n_sp*sqrt(1- req_damp_ratio^2);
poles_required = [complex(a,-b) complex(a,b)]

 % only run to find design points
gain_K = place(A_long_sp, B_long_sp, poles_required);
% -131.9862  -20.3747 for 475 ft/s
K_alpha = gain_K(1); %[deg/rad]
K_q = gain_K(2); %[deg/(rad/s)]


%% gain scheduler
% alpha, q
gain_K_350 = [-126.3240  -26.5162];
gain_K_441 = [-127.265  -21.9864];
gain_K_442 = [-132.3113  -21.8887];
gain_K_600 = [-131.2155  -16.1486];

% gain scheduler pitch rate, always a line
K_q = (velocity - 350)*(gain_K_600(2) - gain_K_350(2))/(600-350) + gain_K_350(2); 


line_switch = 441.5; % ft/s
if velocity <= line_switch
    K_alpha = (velocity - 350)*(gain_K_441(1) - gain_K_350(1))/(441-350) + gain_K_350(1); 
else
    K_alpha = (velocity - 442)*(gain_K_600(1) - gain_K_442(1))/(600-442) + gain_K_442(1); 
end

% K_alpha = -0.001;
% K_q = -0.5;
% gain_K_350 = [-0.001 -0.5];
% gain_K_600 = [-0.001 -0.5];
ctrb(long_sp_ss)
rank(ctrb(long_sp_ss))

%% Gust check
% Are the obtained levels of feedback gains acceptable with reference to
% gust. Mind the units in which Kalpha and Kq are expressed. 
% Consider severe gust (4.572 m/s) according to MIL-F-8785C

gust_vel = 4.572; % m/s vertical gust
alpha_induced = atan(gust_vel/(velocity*ft2m)); % [rad] induced gust angle of attack
velocity
elevator_deflection = K_alpha*alpha_induced % [deg] elevator deflection

%% Required short period frequency and damping ratio
% what procedure can be done to obtain the 
% required short period frequency wnsp and damping ratio?

%% control it
% make a pitch rate command system using such a controller
% for the selected flight condition (10000 ft, 350 ft/s) 
% such that wnsp and damping_ratio have the required value.

% closed loop pitch rate controller using calculated gain K
pr_controller = ss(A_long_sp-B_long_sp*gain_K, B_long_sp, C_long_sp, D_long_sp, ...
                    'StateName', {'alpha', 'q'}, ... 
                     'InputName', {'delta_e'}, ...
                     'OutputName', {'alpha', 'q'});
controller_poles = pole(pr_controller);
tf_q_de_controller       = minreal(tf(pr_controller('q')));
tf_alpha_de_controller   = minreal(tf(pr_controller('alpha')));
% poles values of controlKler
[w_n_sp,damp_ratio_sp] = damp(pr_controller);
w_n_sp_controller    = w_n_sp(1);
damp_ratio_sp_controller  = damp_ratio_sp(1);

%% Step 5
% the Ttheta2 time constant cannot be modified by pole placement or another
% loop structure, has to be done by pole zero cancellation and lead lag
% prefilter. Explain why it must be outside the loop

% lead lag filter simple first order
s = tf('s');
T1 = req_T_theta_2;
T2 = T_theta_2_sys;
tf_lead_lag = (1+T1*s)/(1+T2*s);

tf_q_de_ll = minreal(tf_lead_lag*tf_q_de_controller);
tf_num = cell2mat(tf_q_de_ll.num);
T_theta_2_ll = tf_num(2)/tf_num(3);



%% step 6 Control diagram
% include a feedforward gain such that the steady state pitch rate tracks the input
% Adding a feedforward input to the control allows regulating the gust disturbance,

kff = abs(evalfr(tf_q_de_ll, 0)); % feedforward gain by evaluating the transfer function at 0 frequency
% sisotool(tf_q_de_ll)
tf_q_de_ll2 = tf_q_de_ll/kff; % lead-lag prefilter with feedforward gain
tf_q_de_ll_before_ff = tf_q_de_ll;
tf_q_de_ll = tf_q_de_ll2;

%% step 7 Drawing of CAP
% Give drawings of CAP and Gibson criteria allowable regions together with positions of the
% design point and the current parameter value. Check if requirements are
% met using the right equations.

% DB: dropback, amount of negative transition towards final value after the step input has
% been removed
% OS: overshoot, amount of positive transition towards final value after the step input has
% been removed

%if the pitch rate overshoot ratio qm/qs <= 1 then dropback is not possible
% and the lower part of the "satisfactory” region cannot be attained.
% where • qm: maximum pitch rate • qs: steady state value of pitch rate
% the acceptable values of pitch rate overshoot qm/qs lies in the range 1.0 <= qm/qs <= 3.0

%% CAP before and after 
% CAP_before = gravity*(w_n_sp_sys^2)*T_theta_2_sys/(velocity*ft2m); % design point
CAP_before = w_n_sp_sys^2/((velocity*ft2m)/(gravity*T_theta_2_sys)); % design point is the same
CAP_after = gravity*(w_n_sp_controller^2)*T_theta_2_ll/(velocity*ft2m);

%% gibson dropback criteria DB/q_ss
gibson_dropback_before = T_theta_2_sys - 2*damp_ratio_sp_sys/w_n_sp_sys;
gibson_dropback_after = T_theta_2_ll - 2*damp_ratio_sp_controller/w_n_sp_controller;


%% pitch *rate* time response
figure('Renderer', 'painters', 'Position', [10 10 800 600]);

t_duration = 15;
t = 0:timestep:t_duration;

step_duration = 7; % in seconds
u = @(t) (t < step_duration) * 1 +(t >= step_duration) * 0; % step input using function handle

ss_ts = step_duration/timestep - 1; % timestep at which to grab steady state value: right before the step input finishes

[y_step_before_ll, x_step_before_ll]  = lsim(-1*tf_q_de_controller,u(t),t);
[y_step_after_ll, x_step_after_ll]  = lsim(-1*tf_q_de_ll,u(t),t);

grid on;
hold on;
fontsize(gcf,scale=1.8)

plot(x_step_before_ll, y_step_before_ll);
plot(x_step_after_ll, y_step_after_ll);

% plot(t,u,'black')
plot(t,u(t),'black')
legend("Original Controller", "Controller with filter/feedforward", 'location','northeast')
xlabel("Time [s]")
ylabel("Pitch Rate [deg/s]");

ax = gcf;
exportgraphics(ax,sprintf("figures/ch7_2_step_rate_%0.f.png",velocity),'Resolution',600) 


%% pitch *angle* before and after lead-lag
figure('Renderer', 'painters', 'Position', [10 10 800 600]);

[y_step_before_ll_angle, x_step_before_ll_angle]  = lsim(-1*tf_q_de_controller*(1/s),u(t),t); % multiple by 1/s to get angle
[y_step_after_ll_angle, x_step_after_ll_angle]  = lsim(-1*tf_q_de_ll*(1/s),u(t),t);
grid on;
hold on;
fontsize(gcf,scale=1.8)

plot(x_step_before_ll_angle, y_step_before_ll_angle);
plot(x_step_after_ll_angle, y_step_after_ll_angle);

plot([0,step_duration,t_duration],[0,step_duration,step_duration],'black')

% line that gives the horizontal position of value at step_duration
x = [step_duration,11];
y = [y_step_after_ll_angle(ss_ts+1),y_step_after_ll_angle(ss_ts+1)];
a = plot(x,y,'LineStyle','--','Color',[0.6350 0.0780 0.1840]);
text(11,y_step_after_ll_angle(ss_ts+1),'Dropback','Color', [0.6350 0.0780 0.1840])
% 
% % vertical double arrow with caption dropback
% x = [10,10];
% y = [y_step_after_ll_angle(ss_ts+1),step_duration];
% a = annotation('doublearrow',x,y,'String','Dropback DB');

legend("Original Controller", "Controller with filter/feedforward", 'location','east')
xlabel("Time [s]")
ylabel("Pitch Angle [deg]");

ax = gcf;
exportgraphics(ax,sprintf("figures/ch7_3_step_angle_%0.f.png",velocity),'Resolution',600) 

%% Dropback Criterion
qm_before_ll = max(y_step_before_ll); % max pitch rate
qm_after_ll = max(y_step_after_ll);

qs_before_ll = y_step_before_ll(ss_ts); % pitch rate steady state
qs_after_ll = y_step_after_ll(ss_ts);

pitch_rate_overshoot_ratio_before_ll = qm_before_ll/qs_before_ll;
pitch_rate_overshoot_ratio_after_ll = qm_after_ll/qs_after_ll;

figure('Renderer', 'painters', 'Position', [10 10 800 600]);
% points to paint the blue color
x = [0 0 0.055 0.3];
y = [1 3 3 1];
patch(x,y,'cyan')
% points = [0 0;0 3;0.055 3;0.3 0]; 


% patch(points,'c','FaceColor','cyan');
hold on;
gibson_dropback_after
x1 = scatter(gibson_dropback_before,pitch_rate_overshoot_ratio_before_ll,'filled','magenta','MarkerFaceColor', 'flat');
x2 = scatter(gibson_dropback_after,pitch_rate_overshoot_ratio_after_ll,'filled','black','MarkerFaceColor', 'flat');
xlim([-0.4, 1.2]);
ylim([1, 4]);
fontsize(gcf,scale=1.8)
legend([x1 x2],'Criterion before lead-lag','Criterion after lead-lag','location','northwest');
grid on;
hold on;
text(0,1.5,'satisfactory')
text(0.3,2.3,'abrupt bobble tendency')
text(-0.3,2.3,'Sluggy')
text(0.3,3.3,'Continuous Bobbling tendency')

xlabel('DB/qs [s]')
ylabel('qm/qs [-]')
ax = gcf;
exportgraphics(ax,sprintf("figures/ch7_4_criterion_%0.f.png",velocity),'Resolution',600) 


%% step 8 neighbour
% Now use the same procedure to design a pitch rate command system at a neighbouring flight
% condition in terms of airspeed, with the same altitude. E.g. if your condition is 30000 ft, 600
% ft/s, you can choose 30000 ft 900 ft/s or 30000 ft 300 ft/s. Use the approach of gain scheduling
% to arrive at a controller that is valid across this part of the flight envelope. Evaluate if your
% controller satisfies the Gibson dropback criterion at a velocity in between the two conditions,
% using time responses of pitch attitude angle and pitch rate. Note: when trimming the aircraft
% at a different flight condition, make sure that the trim point is valid!

% so we need to make sure that at 10,000 ft and between 350 and 600 ft/s
% the controller works. Do this by using gain scheduling, i.e., changing
% the gain depending on the operating condition (velocity)
% look at time responses of pitch attitude angle and pitch rate for 475
% feet
%% CAP plots
% phase A
figure('Renderer', 'painters', 'Position', [10 10 900 600]);
fontsize(gcf,scale=1.8)
subplot(1,3,1);

grid on;
hold on;
% Specify pos as a four-element vector of the form [x y w h]
l1 = rectangle('Position',[0.35 0.28 1.4-0.35 3.6-.28],'LineWidth',3,'EdgeColor','black');
l2 = rectangle('Position',[0.25 0.16 2-.25 10-.16],'LineWidth',3,'EdgeColor','black');
l3 = rectangle('Position',[0.15 0.001 13-0.15 13-0.001],'LineWidth',3,'EdgeColor','black');
x1 = scatter(damp_ratio_sp_sys,CAP_before,'filled','magenta');
x2 = scatter(damp_ratio_sp_controller,CAP_after,'filled','black');
set(gca, 'XScale','log')
set(gca, 'YScale','log')
xlim([0.1 10])
ylim([0.01 10]) % mistake in phase A

ylabel('CAP [1/(gsec^{2})]');
xlabel('Damping Ratio [-]');
title('Flight Phase Category A')
legend([x1 x2],'CAP and SP Requirements before lead-lag','CAP and SP Requirements after lead-lag','location','southeast');

% phase B
subplot(1,3,2);

grid on;
hold on;
% Specify pos as a four-element vector of the form [x y w h]
l1 = rectangle('Position',[0.3 0.085 2-0.3 3.6-.085],'LineWidth',3,'EdgeColor','black');
l2 = rectangle('Position',[0.2 0.038 2-.2 10-.038],'LineWidth',3,'EdgeColor','black');
l3 = rectangle('Position',[0.15 0.001 13-0.15 13-0.001],'LineWidth',3,'EdgeColor','black');
x1 = scatter(damp_ratio_sp_sys,CAP_before,'filled','magenta');
x2 = scatter(damp_ratio_sp_controller,CAP_after,'filled','black');
set(gca, 'XScale','log')
set(gca, 'YScale','log')
xlim([0.1 10])
ylim([0.01 10])

ylabel('CAP [1/(gsec^{2})]');
xlabel('Damping Ratio [-]');
title('Flight Phase Category B')

% phase C
subplot(1,3,3);

grid on;
hold on;
% Specify pos as a four-element vector of the form [x y w h]
l1 = rectangle('Position',[0.35 0.16 1.3-0.35 3.6-.16],'LineWidth',3,'EdgeColor','black');
l2 = rectangle('Position',[0.25 0.05 2-.25 10-.05],'LineWidth',3,'EdgeColor','black');
l3 = rectangle('Position',[0.15 0.001 13-0.15 13-0.001],'LineWidth',3,'EdgeColor','black');
x1 = scatter(damp_ratio_sp_sys,CAP_before,'filled','magenta');
x2 = scatter(damp_ratio_sp_controller,CAP_after,'filled','black');
set(gca, 'XScale','log')
set(gca, 'YScale','log')
xlim([0.1 10])
ylim([0.01 10])

ylabel('CAP [1/(gsec^{2})]');
xlabel('Damping Ratio [-]');
title('Flight Phase Category C')
ax = gcf;
exportgraphics(ax,sprintf("figures/ch7_5_CAP_%0.f.png",velocity),'Resolution',600) 
K_alpha
K_q