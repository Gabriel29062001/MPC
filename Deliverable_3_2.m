addpath(fullfile('..', 'src'));

close all
clear all
clc

%% TODO: This file should produce all the plots for the deliverable

% System Initialisation 

Ts = 1/20; % Sample time
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

% Design MPC controller

H = 4; % Horizon length in seconds
Tf = 7

mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_y = MpcControl_y(sys_y, Ts, H);
mpc_z = MpcControl_z(sys_z, Ts, H);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);


%% SYS X
x0 = [0, 0 ,0 ,0]'; 
x_ref = -4;

[u, T_opt, X_opt, U_opt] = mpc_x.get_u(x0, x_ref);
U_opt(:,end+1) = NaN;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_x, xs, us, x_ref); % Plot as usual

[T, X_sub, U_sub] = rocket.simulate_f(sys_x, x0, Tf, @mpc_x.get_u, x_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us, x_ref);


%% SYS Y
x0 = [0, 0 ,0 ,0]';
x_ref = -4;

[u, T_opt, X_opt, U_opt] = mpc_y.get_u(x0, x_ref);
U_opt(:,end+1) = NaN;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_y, xs, us, x_ref); % Plot as usual

[T, X_sub, U_sub] = rocket.simulate_f(sys_y, x0, Tf, @mpc_y.get_u, x_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_y, xs, us, x_ref);

%% SYS Z
x0 = [0,0]';
x_ref = -4;

[u, T_opt, X_opt, U_opt] = mpc_z.get_u(x0, x_ref);
U_opt(:,end+1) = NaN;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_z, xs, us, x_ref); % Plot as usual

[T, X_sub, U_sub] = rocket.simulate_f(sys_z, x0, Tf, @mpc_z.get_u, x_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us, x_ref);

%% SYS ROLL
x0 = [0,0]'; 
x_ref = deg2rad(35);

[u, T_opt, X_opt, U_opt] = mpc_roll.get_u(x0, x_ref);
U_opt(:,end+1) = NaN;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_roll, xs, us, x_ref); % Plot as usual

[T, X_sub, U_sub] = rocket.simulate_f(sys_roll, x0, Tf, @mpc_roll.get_u, x_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us, x_ref);

