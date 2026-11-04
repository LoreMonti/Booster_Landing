function summary = study_landing(figDir)
%STUDY_LANDING Guided landing (step 3): nominal case and single dispersions.
%
%   summary = study_landing(figDir) flies models/landing.slx from the start
%   of the terminal phase (z0 = 5 km, vz0 = -280 m/s), nominally and with
%   one error at a time that the guidance does not know about: thrust
%   -5 %, drag coefficient 0.4 and 1.2, Isp -5 %. It saves
%     figures/landing_nominal.png     altitude, velocity, thrust, mass;
%     figures/landing_dispersions.png velocity against altitude for all cases,
%   prints the touchdown table and returns it.

cases = {'Nominal', {}; 'Thrust -5 %', {'kT', 0.95}; 'Thrust -10 %', {'kT', 0.90}; ...
         'C_D = 0.4', {'CD', 0.4}; 'C_D = 1.2', {'CD', 1.2}; 'I_{sp} -5 %', {'Isp', 0.95*282}; ...
         'Combined', {'kT', 0.95, 'CD', 0.4, 'Isp', 0.95*282}};
p0 = params();
runs = cell(size(cases, 1), 1);
for i = 1:size(cases, 1)
    p = p0;
    for j = 1:2:numel(cases{i, 2})
        p.(cases{i, 2}{j}) = cases{i, 2}{j + 1};
    end
    runs{i} = simulate_landing(p, 120);
end

% Figure 1: the nominal landing
s  = runs{1};
on = s.T > 0;
iIgn = find(on, 1);
fig = figure('Visible', 'off', 'Theme', 'light', 'Position', [100 100 900 760]);
tl = tiledlayout(fig, 4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf(['Guided landing, nominal: ignition at %.0f m, touchdown at %.2f m/s ' ...
    'after %.1f s, %.0f kg of propellant left'], s.z(iIgn), s.vz(end), s.t(end), s.m(end) - p0.mdry));
ax = gobjects(4, 1);
for k = 1:4, ax(k) = nexttile(tl); hold(ax(k), 'on'); grid(ax(k), 'on'); end
plot(ax(1), s.t, s.z/1e3, 'LineWidth', 1.4);       ylabel(ax(1), 'z [km]');
plot(ax(2), s.t, s.vz, 'LineWidth', 1.4);          ylabel(ax(2), 'v_z [m/s]');
plot(ax(3), s.t, s.T/1e3, 'LineWidth', 1.4);       ylabel(ax(3), 'T [kN]');
yline(ax(3), [p0.Tmin p0.Tmax]/1e3, '--', {'T_{min}', 'T_{max}'}, 'HandleVisibility', 'off');
plot(ax(4), s.t, s.m/1e3, 'LineWidth', 1.4);       ylabel(ax(4), 'm [t]');
yline(ax(4), p0.mdry/1e3, ':', 'dry mass', 'HandleVisibility', 'off');
for k = 1:4, xline(ax(k), s.t(iIgn), '--', 'ignition', 'HandleVisibility', 'off'); end
xlabel(ax(4), 't [s]');
exportgraphics(fig, fullfile(figDir, 'landing_nominal.png'), 'Resolution', 150);
close(fig);

% Figure 2: phase plane of every case, against the full-thrust hoverslam
fig = figure('Visible', 'off', 'Theme', 'light', 'Position', [100 100 900 560]);
ax = axes(fig);  hold(ax, 'on');  grid(ax, 'on');
cols = lines(numel(runs));
for i = 1:numel(runs)
    plot(ax, runs{i}.z/1e3, -runs{i}.vz, 'Color', cols(i, :), 'LineWidth', 1.3, ...
        'DisplayName', cases{i, 1});
end
h = hoverslam_ignition(p0, 280);
plot(ax, h/1e3, 280, 'kp', 'MarkerSize', 12, 'MarkerFaceColor', 'k', ...
    'DisplayName', sprintf('full-thrust ignition, no drag (%.0f m)', h));
set(ax, 'XDir', 'reverse');
xlabel(ax, 'altitude z [km]');  ylabel(ax, 'descent speed -v_z [m/s]');
title(ax, sprintf(['Guided descents with drag feedforward and integral action ' ...
    '(\\eta = %.1f, K_i = %g s^{-1})'], p0.eta, p0.KiGuid));
legend(ax, 'Location', 'northwest');
exportgraphics(fig, fullfile(figDir, 'landing_dispersions.png'), 'Resolution', 150);
close(fig);

% Touchdown table
n = numel(runs);
[tTd, vz, vx, x, th, prop, zIgn, Tlo, Thi] = deal(zeros(n, 1));
for i = 1:n
    s = runs{i};  on = s.T > 0;
    tTd(i) = s.t(end);  vz(i) = s.vz(end);  vx(i) = s.vx(end);  x(i) = s.x(end);
    th(i) = rad2deg(s.theta(end));  prop(i) = s.m(end) - p0.mdry;
    zIgn(i) = s.z(find(on, 1));  Tlo(i) = min(s.T(on))/1e3;  Thi(i) = max(s.T(on))/1e3;
end
summary = table(string(cases(:, 1)), zIgn, tTd, vz, vx, x, th, prop, Tlo, Thi, ...
    'VariableNames', {'Case', 'Ignition_m', 'Touchdown_s', 'Vz_mps', 'Vx_mps', 'X_m', ...
    'Theta_deg', 'PropellantLeft_kg', 'Tmin_kN', 'Tmax_kN'});
disp(summary);
end

function s = simulate_landing(p, tEnd)
% One run of models/landing.slx; returns its nine outputs.
in = Simulink.SimulationInput('landing');
in = in.setVariable('p', p);
in = in.setModelParameter('Solver', 'ode45', 'RelTol', '1e-8', 'AbsTol', '1e-8', ...
    'StopTime', num2str(tEnd), 'SaveOutput', 'on', 'OutputSaveName', 'yout', ...
    'SaveFormat', 'Dataset');
out = sim(in);
names = {'x', 'z', 'vx', 'vz', 'theta', 'q', 'm', 'delta', 'T'};
for i = 1:numel(names)
    s.(names{i}) = out.yout{i}.Values.Data;
end
s.t = out.yout{1}.Values.Time;
end
