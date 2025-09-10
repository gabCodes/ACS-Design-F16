FindF16Dynamics;
A_long = SS_lo.A([3 7 8 5 11], [3 7 8 5 11]);
B_long = SS_lo.A([3 7 8 5 11], [13, 14]);
C_long = SS_lo.C([3 7 8 5 11], [3 7 8 5 11]);
D_long = SS_lo.C([3 7 8 5 11], [13, 14]);

%% Parameters
x1 = 800;
x2 = 764.1;
tau = 3.15;
hf = 49.5;
ho = 9.44;
touchdowntime = 5.21;

sim GS_Flare;
%Have to run the block diagram in simulink to get these outputs for
%plotting, paste in command window after for plots

% figure();
% plot(out.Airspeed);
% grid on;
% title('Airspeed Timeseries');
% xlabel('Time (s)');
% ylabel('Airspeed change from trim conditions (ft/s)');
% 
% figure();
% plot(out.AoR);
% grid on;
% title('Altitude over runway timeseries');
% xlabel('Time (s)');
% ylabel('Altitude over runway (ft)');
% 
% figure();
% plot(out.GSErr);
% grid on;
% title('Glideslope error timeseries');
% xlabel('Time (s)');
% ylabel('Glideslope error (deg)');
% 
% figure();
% plot(out.Vs);
% grid on;
% title('Vertical speed timeseries');
% xlabel('Time (s)');
% ylabel('Vertical speed (ft/s)');
% 
% figure();
% plot(out.halt);
% grid on;
% title('Altitude timeseries');
% xlabel('Time (s)');
% ylabel('Altitude change from trim (ft)');
% 
% figure();
% plot(out.mtheta);
% grid on;
% title('Theta timeseries');
% xlabel('Time (s)');
% ylabel('Theta change from trim (deg)');



