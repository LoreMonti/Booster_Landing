function res = study_attitude(figDir)
%STUDY_ATTITUDE Margins and time response of the pitch loop (step 2).
%
%   res = study_attitude(figDir)
%   1. Linear analysis: after the dynamic inversion the open loop is
%      L(s) = (Kd*s + Kp)/s^2 * 1/(tauAct*s + 1) at every throttle
%      setting. Its phase margin, crossover and delay margin are printed
%      and figures/attitude_margins.png shows the Bode plot.
%   2. Nonlinear check on models/attitude_loop.slx: a 3 deg tilt at Tmax
%      and at Tmin against the ideal second-order response, and a 10 deg
%      step at Tmin that saturates the gimbal
%      (figures/attitude_response.png).
%   res is a struct with the margins and the settling times.

p = params();

% 1. Linear margins
s = tf('s');
L = (p.KdAtt*s + p.KpAtt)/s^2/(p.tauAct*s + 1);
[gm, pm, ~, wc] = margin(L);
res.phaseMargin_deg = pm;
res.gainMargin_dB   = 20*log10(gm);
res.crossover_rad_s = wc;
res.delayMargin_s   = deg2rad(pm)/wc;

fig = figure('Visible', 'off', 'Theme', 'light', 'Position', [100 100 800 560]);
opts = bodeoptions;  opts.FreqUnits = 'rad/s';  opts.Grid = 'on';
margin(L, opts);
title(sprintf(['Pitch loop after inversion: phase margin %.0f deg at %.2f rad/s, ' ...
    'delay margin %.2f s'], pm, wc, res.delayMargin_s));
exportgraphics(fig, fullfile(figDir, 'attitude_margins.png'), 'Resolution', 150);
close(fig);

% 2. Nonlinear time responses
runs = struct('name', {}, 's', {}, 'ref', {});
for name = {'Tmax', 'Tmin'}
    q = p;  q.T = q.(name{1});  q.theta0 = deg2rad(3);
    runs(end+1) = struct('name', sprintf('3 deg tilt, T = %s', name{1}), ...
        's', simulate_attitude(q, 10), 'ref', q.thetaRef); %#ok<AGROW>
end
q = p;  q.T = q.Tmin;  q.thetaRef = deg2rad(10);
runs(end+1) = struct('name', '10 deg step, T = Tmin', 's', simulate_attitude(q, 10), 'ref', q.thetaRef);

% ideal free response of s^2 + Kd*s + Kp from 3 deg
t  = linspace(0, 10, 500)';
z  = p.zetaAtt;  w = p.wnAtt;  wd = w*sqrt(1 - z^2);
th = deg2rad(3)*exp(-z*w*t).*(cos(wd*t) + z/sqrt(1 - z^2)*sin(wd*t));

fig = figure('Visible', 'off', 'Theme', 'light', 'Position', [100 100 900 640]);
tl = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf('Pitch loop on the 3-DOF plant (\\omega_n = %g rad/s, \\zeta = %g)', w, z));
ax1 = nexttile(tl);  hold(ax1, 'on');
ax2 = nexttile(tl);  hold(ax2, 'on');
plot(ax1, t, rad2deg(th), 'k:', 'LineWidth', 1.6, 'DisplayName', 'ideal 2nd order, 3 deg');
cols = lines(numel(runs));      % same colour for a run in both panels
for i = 1:numel(runs)
    plot(ax1, runs(i).s.t, rad2deg(runs(i).s.theta), 'Color', cols(i, :), ...
        'LineWidth', 1.3, 'DisplayName', runs(i).name);
    plot(ax2, runs(i).s.t, rad2deg(runs(i).s.delta), 'Color', cols(i, :), ...
        'LineWidth', 1.3, 'DisplayName', runs(i).name);
    settle = find(abs(runs(i).s.theta - runs(i).ref) > deg2rad(0.5), 1, 'last');
    res.settling_s.(sprintf('run%d', i)) = runs(i).s.t(min(settle + 1, numel(runs(i).s.t)));
end
yline(ax2, rad2deg([-p.deltaMax p.deltaMax]), '--', 'gimbal limit', 'HandleVisibility', 'off');
ylabel(ax1, '\theta [deg]');  ylabel(ax2, '\delta [deg]');  xlabel(ax2, 't [s]');
legend(ax1, 'Location', 'east');  grid(ax1, 'on');  grid(ax2, 'on');
exportgraphics(fig, fullfile(figDir, 'attitude_response.png'), 'Resolution', 150);
close(fig);

disp(res);  disp(res.settling_s);
end

function s = simulate_attitude(p, tEnd)
% One run of models/attitude_loop.slx; returns its eight outputs.
in = Simulink.SimulationInput('attitude_loop');
in = in.setVariable('p', p);
in = in.setModelParameter('Solver', 'ode45', 'RelTol', '1e-8', 'AbsTol', '1e-8', ...
    'StopTime', num2str(tEnd), 'SaveOutput', 'on', 'OutputSaveName', 'yout', ...
    'SaveFormat', 'Dataset');
out = sim(in);
names = {'x', 'z', 'vx', 'vz', 'theta', 'q', 'm', 'delta'};
for i = 1:numel(names)
    s.(names{i}) = out.yout{i}.Values.Data;
end
s.t = out.yout{1}.Values.Time;
end
