%% TODO: This file should produce all the plots for the deliverable

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

mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_y = MpcControl_y(sys_y, Ts, H);
mpc_z = MpcControl_z(sys_z, Ts, H);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);

% Merge four sub−system controllers into one full−system controller
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);


rocket.mass = 2.13;

%% Without mass rate

x0 = [zeros(1, 9), 1 0 3]'
ref = [1.2, 0, 3, 0]'
Tf = 8

[T, X, U, Ref] = rocket.simulate_est_z(x0, Tf, @mpc.get_u, ref, mpc_z, sys_z);
rocket.anim_rate = 5; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Merged lin. MPC in nonlinear simulation without mass rate '; % Set a figure title

% Setup reference function
ref = @(t , x ) ref_TVC(t );
% Simulate
Tf = 30;
x0 = zeros(12,1);

[T, X, U, Ref] = rocket.simulate_est_z(x0, Tf, @mpc.get_u, ref, mpc_z, sys_z);
% Visualize
rocket.anim_rate = 5; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Merged lin. MPC in nonlinear simulation without mass rate'; % Set a figure title

%% With mass rate

% Setup reference function
ref = @(t , x ) ref_TVC(t );
% Simulate
Tf = 30;
x0 = zeros(12,1);

rocket.mass_rate = -0.27;
[T, X, U, Ref] = rocket.simulate_est_z(x0, Tf, @mpc.get_u, ref, mpc_z, sys_z);
% Visualize
rocket.anim_rate = 5; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Merged lin. MPC in nonlinear simulation with mass rate'; % Set a figure title


x0 = [zeros(1, 9), 1 0 3]'
ref = [1.2, 0, 3, 0]'
Tf = 8

[T, X, U, Ref] = rocket.simulate_est_z(x0, Tf, @mpc.get_u, ref, mpc_z, sys_z);
rocket.anim_rate = 5; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Merged lin. MPC in nonlinear simulation with mass rate '; % Set a figure title



