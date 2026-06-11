function p = wind_plant_parameters()
%WIND_PLANT_PARAMETERS Parameters for the benchmark DFIG WPS LFC plant.
%
% Values follow Table I and the plant equations in:
% "Observer-Based Fuzzy Event-Triggered LFC for Wind Power Systems Under
% Multiple Cyberattacks via an NN Approximation Approach".

p.Cd = 0.015;       % pu/Hz
p.Mi = 0.1667;      % pu*s
p.Tg = 0.08;        % s
p.Tt = 0.4;         % s
p.Cs = 3;           % Hz/pu
p.beta = 0.3483;    % pu
p.rho = 1.225;      % kg/m^3
p.R = 5;            % m
p.pitch = 0;        % deg
p.phi_opt = 8.1;    % pu
p.Cf = 150;         % pu
p.Ci = 0.1;         % s
p.Lm = 52;          % pu
p.Ls = 0.07397;     % pu
p.Lr = 0.002;       % pu
p.Ss = 1;           % m/s
p.Rs = 7.9;         % pu
p.Rr = 2;           % pu
p.Rw_max = 1.8;     % m/s
p.Rw_min = -1.8;    % m/s

% Simulation settings printed below Table I.
p.tau = 0.01;
p.h = 0.01;
p.tau_bar = p.tau + p.h;
p.epsilon = 0.01;
p.kappa = 2;
p.gamma = 2;
p.varrho1 = 0.05;
p.varrho2 = 0.5;
p.o1 = 0.5;
p.o2 = 0.5;
p.delta_min = 0.01;
p.delta_max = 0.1;
p.iota = 50;
p.sigma = 5;
p.T = [0, 50];
p.x0 = [0.1; -0.2; 0.2; 0; 0.1; -0.2];

% DFIG derived constants.
p.Lrm = p.Lr + p.Lm;
p.Lsm = p.Ls + p.Lm;
p.Lstar = p.Lrm + p.Lm^2 / p.Lsm;
p.W1 = p.Lstar / (p.Ss * p.Rs);
p.W2 = p.Lm / p.Lsm;
p.Cp = (0.44 - 0.0167 * p.pitch) * ...
    sin(pi * (p.phi_opt - 0.2) / (13 - 0.3 * p.pitch)) - ...
    0.00184 * (p.phi_opt - 2) * p.pitch;
p.Pm = 0.5 * p.rho * pi * p.R^5 * p.Cp / p.phi_opt^3;

% Local T-S matrices at Rw_min and Rw_max.
p.A1 = local_A(p, p.Rw_min);
p.A2 = local_A(p, p.Rw_max);
p.B = [0; 1 / p.Tg; 0; 0; 1 / (p.W1 * p.Rr); 0];
p.E = [-1 / p.Mi; 0; 0; 0; 0; 0];
p.C = [p.beta, 0, 0, 0, 0, 0;
       0,      1, 0, 0, 0, 0;
       0,      0, 0, 1, 0, 0];
end

function A = local_A(p, Rw)
A = zeros(6, 6);
A(1, 1) = -p.Cd / p.Mi;
A(1, 3) = 1 / p.Mi;
A(1, 5) = -p.W2 * Rw / p.Mi;
A(1, 6) = 1 / p.Mi;

A(2, 1) = -1 / (p.Cs * p.Tg);
A(2, 2) = -1 / p.Tg;

A(3, 2) = 1 / p.Tt;
A(3, 3) = -1 / p.Tt;

A(4, 1) = p.beta;

A(5, 5) = -1 / p.W1;

A(6, 5) = -p.W2 / (2 * p.Ci);
A(6, 6) = p.Pm * Rw / (2 * p.Ci) - p.Cf;
end
