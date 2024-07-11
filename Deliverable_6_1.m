addpath(fullfile('..', 'src'));

close all
clear all
clc

%% TODO: This file should produce all the plots for the deliverable

Ts = 1/20;
rocket = Rocket(Ts);
H = 1; % Horizon length in seconds
nmpc = NmpcControl(rocket, H);


x0 = zeros(12,1);
ref = [0.5, 0, 1, deg2rad(65)]'
% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state
[u, T_opt, X_opt, U_opt] = nmpc.get_u(x0, ref);
U_opt(:,end+1) = nan;
ph = rocket.plotvis(T_opt, X_opt, U_opt, ref);
% MPC reference with default maximum roll = 15 deg

ref = @(t , x ) ref_TVC(t );
% MPC reference with specified maximum roll = 50 deg
roll_max = deg2rad(50);
ref = @(t , x ) ref_TVC(t , roll_max);

Tf = 30;
rocket.mass = 1.75;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
rocket.anim_rate = 10;
ph = rocket.plotvis(T, X, U,Ref);