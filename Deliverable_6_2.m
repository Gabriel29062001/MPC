addpath(fullfile('..', 'src'));

close all
clear all
clc

Ts = 1/40; % Higher sampling rate for this part!

... Define NMPC ...

rocket = Rocket(Ts);
H = 1; % Horizon length in seconds


rocket.delay = 8;
x0 = zeros(12, 1);
ref = [0.5, 0, 1, deg2rad(65)]';
Tf = 2.5;
rocket.mass = 1.75;

%% With full delay compensated
expected_delay = 8;
nmpc = NmpcControl(rocket, H,expected_delay);
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
rocket.anim_rate = 10;
ph = rocket.plotvis(T, X, U,Ref);

%% With partial delay compensated

expected_delay = 6;
nmpc = NmpcControl(rocket, H,expected_delay);
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
rocket.anim_rate = 10;
ph = rocket.plotvis(T, X, U,Ref);