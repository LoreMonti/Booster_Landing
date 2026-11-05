function f = study_failures(figDir)
%STUDY_FAILURES Failures injected into the guided landing (extension 4).
%
%   f = study_failures(figDir) flies models/landing.slx with the failures
%   of params.m (p.fail), one at a time, from the nominal initial state:
%
%   Thrust loss  a fraction dT of the thrust lost tau seconds after the
%                nominal ignition, on a grid of dT and tau. The tolerated
%                loss is compared with the static margin
%                1 - max(T(t >= tau))/Tmax, the loss the nominal thrust of
%                the rest of the burn could absorb if the stage did not
%                fall behind its plan.
%   Altimeter    a constant bias zBias on the altitude read by the
%                guidance, against the closed form of the impact speed
%                sqrt(vTouch^2 + 2*aStar*zBias) for a positive bias.
%   Velocity     a drift vxDrift*t on the lateral velocity read by the
%                lateral law and the observer, with the observer on and off.
%
%   It saves figures/failures_thrust.png and figures/failures_sensors.png
%   and returns the grids and tables in f.

p0 = params();
nom = fly(p0);
tIgn = nom.t(find(nom.T > 0, 1));
tBurn = nom.t(end) - tIgn;
fprintf('Nominal: ignition at %.2f s, burn %.1f s\n', tIgn, tBurn);

% --- Thrust loss: grid of the time after ignition and of the fraction lost
tau = 0:2:24;
dT  = 0:0.025:0.4;
[TT, DD] = meshgrid(tau, dT);
in = arrayfun(@(k) setup(p0, {'fail.tT', tIgn + TT(k), 'fail.dT', DD(k)}), 1:numel(TT));
r = run_all_inputs(in, p0);
f.thrust.tau = tau;  f.thrust.dT = dT;
f.thrust.vz = reshape(r.vz, size(TT));  f.thrust.ok = reshape(r.ok, size(TT));
% largest loss with every smaller loss also landing
f.thrust.tolerated = arrayfun(@(j) dT(find(~[f.thrust.ok(:, j); false], 1) - 1), 1:numel(tau));
f.thrust.static = arrayfun(@(t0) 1 - max(nom.T(nom.t >= tIgn + t0))/p0.Tmax, tau);
disp(array2table([tau' 100*f.thrust.tolerated' 100*f.thrust.static'], ...
    'VariableNames', {'tau_s', 'Tolerated_pct', 'StaticMargin_pct'}));

fig = figure('Visible', 'off', 'Theme', 'light', 'Position', [100 100 900 520]);
ax = axes(fig);  hold(ax, 'on');
imagesc(ax, tau, 100*dT, log10(abs(f.thrust.vz)));  axis(ax, 'xy');
cb = colorbar(ax);  cb.Label.String = 'touchdown speed |v_z| [m/s]';
cb.Ticks = log10([1 2 5 10 20 50 100]);  cb.TickLabels = {'1', '2', '5', '10', '20', '50', '100'};
clim(ax, log10([1 150]));  colormap(ax, flipud(hot(256)));
contour(ax, tau, 100*dT, double(f.thrust.ok), [0.5 0.5], 'k', 'LineWidth', 2, 'DisplayName', 'edge of success');
plot(ax, tau, 100*f.thrust.static, 'b--', 'LineWidth', 1.5, 'DisplayName', 'static margin 1 - max_{t\geq\tau} T/T_{max}');
xlabel(ax, 'time of the failure after ignition \tau [s]');  ylabel(ax, 'thrust lost [%]');
title(ax, sprintf('Sudden thrust loss during a %.0f s burn: touchdown speed', tBurn));
legend(ax, 'Location', 'northeast');  xlim(ax, [tau(1) tau(end)]);  ylim(ax, 100*[dT(1) dT(end)]);
exportgraphics(fig, fullfile(figDir, 'failures_thrust.png'), 'Resolution', 150);
close(fig);

% --- Sensor faults
zb = [-20 -10 -5 -2 -1 -0.5 0 0.1 0.25 0.5 1 2 5 10 20];
in = arrayfun(@(b) setup(p0, {'fail.zBias', b}), zb);
r = run_all_inputs(in, p0);
f.altimeter = table(zb', r.vz, r.ign, r.prop, r.ok, 'VariableNames', ...
    {'zBias_m', 'vz_mps', 'Ignitions', 'PropellantLeft_kg', 'Success'});
disp(f.altimeter);

vd = -0.2:0.05:0.2;
in = [arrayfun(@(b) setup(p0, {'fail.vxDrift', b, 'kDob', 1}), vd), ...
      arrayfun(@(b) setup(p0, {'fail.vxDrift', b, 'kDob', 0}), vd)];
r = run_all_inputs(in, p0);
n = numel(vd);
f.drift = table(vd', r.x(1:n), r.vx(1:n), r.x(n+1:end), r.vx(n+1:end), 'VariableNames', ...
    {'vxDrift_mps2', 'xOn_m', 'vxOn_mps', 'xOff_m', 'vxOff_mps'});
disp(f.drift);

fig = figure('Visible', 'off', 'Theme', 'light', 'Position', [100 100 900 420]);
tl = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, 'Sensor faults: the guidance and the lateral law fly on measured signals');
ax = nexttile(tl);  hold(ax, 'on');  grid(ax, 'on');
hb = linspace(0, max(zb), 200);
plot(ax, hb, sqrt(p0.vTouch^2 + 2*p0.aStar*hb), 'k--', 'DisplayName', '(u_t^2 + 2a^*h_b)^{1/2}');
plot(ax, zb, abs(f.altimeter.vz_mps), 'o-', 'LineWidth', 1.4, 'DisplayName', 'simulated');
yline(ax, p0.req.vz, ':', 'requirement', 'HandleVisibility', 'off');
yyaxis(ax, 'right');  bar(ax, zb(zb < 0), f.altimeter.Ignitions(zb < 0), 0.4, 'FaceAlpha', 0.4, 'DisplayName', 'ignitions');
ylabel(ax, 'ignitions');  yyaxis(ax, 'left');
xlabel(ax, 'altimeter bias h_b (measured - true) [m]');  ylabel(ax, 'touchdown speed |v_z| [m/s]');
title(ax, 'Altimeter bias');  legend(ax, 'Location', 'north');
ax = nexttile(tl);  hold(ax, 'on');  grid(ax, 'on');
plot(ax, vd, f.drift.xOff_m, 's-', 'LineWidth', 1.4, 'DisplayName', 'observer off');
plot(ax, vd, f.drift.xOn_m, 'o-', 'LineWidth', 1.4, 'DisplayName', 'observer on');
yline(ax, [-p0.req.x p0.req.x], ':', 'HandleVisibility', 'off');
xlabel(ax, 'drift of the measured v_x [m/s^2]');  ylabel(ax, 'x at touchdown [m]');
title(ax, 'Lateral velocity drift');  legend(ax, 'Location', 'northeast');
exportgraphics(fig, fullfile(figDir, 'failures_sensors.png'), 'Resolution', 150);
close(fig);
end

function in = setup(p0, kv)
% SimulationInput for p0 with the fields of the name/value list changed
% (dotted names such as 'fail.dT' reach the sub-struct).
p = p0;
for j = 1:2:numel(kv)
    path = strsplit(kv{j}, '.');
    p = setfield(p, path{:}, kv{j + 1});
end
in = Simulink.SimulationInput('landing');
in = in.setVariable('p', p);
in = in.setModelParameter('Solver', 'ode45', 'RelTol', '1e-8', 'AbsTol', '1e-8', ...
    'StopTime', '120', 'SaveOutput', 'on', 'OutputSaveName', 'yout', 'SaveFormat', 'Dataset');
end

function r = run_all_inputs(in, p0)
% Flies every input and collects the touchdown state.
out = sim(in, 'ShowProgress', 'off', 'ShowSimulationManager', 'off');
n = numel(out);
[r.x, r.vx, r.vz, r.prop, r.ign, r.t] = deal(nan(n, 1));
for i = 1:n
    y = out(i).yout;
    v = @(k) y{k}.Values.Data;
    x = v(1);  vx = v(3);  vz = v(4);  m = v(7);  T = v(9);
    r.x(i) = x(end);  r.vx(i) = vx(end);  r.vz(i) = vz(end);
    r.prop(i) = m(end) - p0.mdry;  r.ign(i) = sum(diff(T > 0) > 0);
    r.t(i) = y{1}.Values.Time(end);
end
q = p0.req;
% an engine relit tens of times does not count as a landing
r.ok = r.t < 120 & abs(r.vz) < q.vz & abs(r.vx) < q.vx & abs(r.x) < q.x & r.prop > 0 & r.ign == 1;
end

function s = fly(p)
% One run of models/landing.slx; returns x, vz, T, t.
out = sim(setup(p, {}));
s.t = out.yout{1}.Values.Time;  s.T = out.yout{9}.Values.Data;  s.vz = out.yout{4}.Values.Data;
end
