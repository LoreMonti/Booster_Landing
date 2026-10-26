function errs = study_plant_validation(figDir)
%STUDY_PLANT_VALIDATION Error of the 3-DOF plant against four closed-form solutions.
%
%   errs = study_plant_validation(figDir) runs the four cases of
%   TestPlant3DOF (each switches on a single term of the dynamics),
%   plots the absolute error against the exact solution on a log scale
%   and saves figures/plant_validation.png. errs is a table with the
%   maximum error of each case.

relTol = 1e-10;
cases = {};

% 1. Upright, no drag: the rocket equation with gravity
p = params();  p.CD = 0;
s = simulate_plant(p, 15, relTol);
[mRef, L, k, c] = rocket(p, s.t);
zRef = p.z0 + p.vz0*s.t - 0.5*p.g*s.t.^2 + c*(s.t - (mRef/k).*L);
cases(end+1, :) = {'Upright thrust: z', 'm', s.t, abs(s.z - zRef)};

% 2. Fixed tilt, no gravity, no drag: thrust along theta0
p = params();  p.g = 0;  p.CD = 0;  p.theta0 = deg2rad(10);
s = simulate_plant(p, 15, relTol);
[mRef, L, k, c] = rocket(p, s.t);
xRef = p.x0 + p.vx0*s.t + c*(s.t - (mRef/k).*L)*sin(p.theta0);
cases(end+1, :) = {'Tilted thrust: x', 'm', s.t, abs(s.x - xRef)};

% 3. Constant gimbal: constant pitch acceleration
p = params();  p.delta = deg2rad(1);
s = simulate_plant(p, 5, relTol);
alpha = -p.T*p.l*sin(p.delta)/p.I;
thRef = p.theta0 + p.q0*s.t + 0.5*alpha*s.t.^2;
cases(end+1, :) = {'Gimbal torque: \theta', 'rad', s.t, abs(s.theta - thRef)};

% 4. Unpowered fall in constant density: tanh approach to terminal velocity
p = params();  p.T = 0;  p.H = Inf;  p.vz0 = -100;
s = simulate_plant(p, 10, relTol);
vt = sqrt(p.g/(p.rho0*p.CD*p.A/(2*p.m0)));
vzRef = -vt*tanh(p.g*s.t/vt + atanh(abs(p.vz0)/vt));
cases(end+1, :) = {'Drag-limited fall: v_z', 'm/s', s.t, abs(s.vz - vzRef)};

% Figure: one panel per case
fig = figure('Visible', 'off', 'Theme', 'light', 'Position', [100 100 900 620]);
tl = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf('3-DOF plant against closed-form solutions (ode45, RelTol = AbsTol = %g)', relTol));
for i = 1:size(cases, 1)
    ax = nexttile(tl);
    semilogy(ax, cases{i, 3}, max(cases{i, 4}, eps), 'LineWidth', 1.2);
    grid(ax, 'on');
    title(ax, cases{i, 1});
    xlabel(ax, 't [s]');
    ylabel(ax, sprintf('absolute error [%s]', cases{i, 2}));
    ylim(ax, [1e-16 1e-6]);
end
exportgraphics(fig, fullfile(figDir, 'plant_validation.png'), 'Resolution', 150);
close(fig);

errs = table(cases(:, 1), cellfun(@max, cases(:, 4)), cases(:, 2), ...
    'VariableNames', {'Case', 'MaxAbsError', 'Unit'});
disp(errs);
end

function [mRef, L, k, c] = rocket(p, t)
% Mass and log term of the rocket equation at constant thrust.
k    = p.T/(p.Isp*p.g0);    % propellant mass flow [kg/s]
c    = p.Isp*p.g0;          % effective exhaust velocity [m/s]
mRef = p.m0 - k*t;
L    = log(p.m0./mRef);
end
