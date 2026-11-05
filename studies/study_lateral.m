function [summary, wind] = study_lateral(figDir)
%STUDY_LATERAL Lateral control: PD against LQR (step 4), then wind (extension 2).
%
%   [summary, wind] = study_lateral(figDir) flies models/landing.slx.
%
%   Step 4 compares the two lateral gain sets of params.m, which share the
%   attitude controller, the gimbal and the guidance and differ only in
%   the four gains K of
%       thetaRef = -(K1 x + K2 vx + kDob*dHat)/a + K3 theta + K4 q,
%   with the disturbance observer off (kDob = 0), so that only the gains
%   differ. Scenarios: four lateral errors at the start of the terminal
%   phase, each nominal and with the combined thrust, drag and Isp error.
%   It saves figures/lateral_comparison.png (x, theta and delta for the
%   30 m / 2 m/s case) and returns the table in summary.
%
%   Extension 2 flies the LQR in a wind, with the observer off and on,
%   then sweeps kDob and tauDob on the case of TestLateral (30 m, 2 m/s,
%   15 m/s wind, thrust 5 % low, CD = 1.2). It saves
%   figures/lateral_wind.png and returns the tables in wind.

offsets = [30 2; -30 -2; 50 0; 0 3];    % [x0 [m], vx0 [m/s]]
gains   = {'KlatPD', 'Cascaded PD'; 'KlatLQR', 'LQR'};
errs    = {'nominal', {}; 'combined', {'kT', 0.95, 'CD', 0.4, 'Isp', 0.95*282}};

rows = {};
example = cell(1, 2);
for g = 1:size(gains, 1)
    for o = 1:size(offsets, 1)
        for e = 1:size(errs, 1)
            p = params();
            p.Klat = p.(gains{g, 1});
            p.kDob = 0;
            p.x0 = offsets(o, 1);  p.vx0 = offsets(o, 2);
            for j = 1:2:numel(errs{e, 2}), p.(errs{e, 2}{j}) = errs{e, 2}{j + 1}; end
            s = simulate_landing(p, 120);
            on = s.T > 0;  ig = find(on, 1);
            late = find(abs(s.x) > 1, 1, 'last');
            rows(end+1, :) = {gains{g, 2}, sprintf('%+g m, %+g m/s', offsets(o, :)), errs{e, 1}, ...
                abs(s.x(end)), abs(s.vx(end)), rad2deg(max(abs(s.theta))), ...
                rad2deg(max(abs(s.delta))), s.t(min(late + 1, end)) - s.t(ig), s.m(end) - p.mdry}; %#ok<AGROW>
            if o == 1 && e == 1, example{g} = s; end
        end
    end
end
summary = cell2table(rows, 'VariableNames', {'Controller', 'Offset', 'Errors', 'Miss_m', ...
    'Vx_mps', 'PeakTilt_deg', 'PeakGimbal_deg', 'TimeTo1m_s', 'PropellantLeft_kg'});
disp(summary);

% Figure: the 30 m, 2 m/s case, both controllers
p = params();
fig = figure('Visible', 'off', 'Theme', 'light', 'Position', [100 100 900 720]);
tl = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, 'Lateral correction from 30 m and 2 m/s off the pad: same actuator, different gains');
ax = gobjects(3, 1);
for k = 1:3, ax(k) = nexttile(tl); hold(ax(k), 'on'); grid(ax(k), 'on'); end
cols = lines(2);
for g = 1:2
    s = example{g};
    plot(ax(1), s.t, s.x, 'Color', cols(g, :), 'LineWidth', 1.4, 'DisplayName', gains{g, 2});
    plot(ax(2), s.t, rad2deg(s.theta), 'Color', cols(g, :), 'LineWidth', 1.4, 'HandleVisibility', 'off');
    plot(ax(3), s.t, rad2deg(s.delta), 'Color', cols(g, :), 'LineWidth', 1.4, 'HandleVisibility', 'off');
end
yline(ax(1), [-p.req.x p.req.x], ':', 'HandleVisibility', 'off');
yline(ax(3), rad2deg([-p.deltaMax p.deltaMax]), '--', 'HandleVisibility', 'off');
ign = example{1}.t(find(example{1}.T > 0, 1));
for k = 1:3, xline(ax(k), ign, '--', 'ignition', 'HandleVisibility', 'off'); end
ylabel(ax(1), 'x [m]');  ylabel(ax(2), '\theta [deg]');  ylabel(ax(3), '\delta [deg]');
xlabel(ax(3), 't [s]');  legend(ax(1), 'Location', 'northeast');
exportgraphics(fig, fullfile(figDir, 'lateral_comparison.png'), 'Resolution', 150);
close(fig);

wind = study_wind(figDir);
end

function w = study_wind(figDir)
% Extension 2: the LQR with and without the disturbance observer.
accept = {'x0', 30, 'vx0', 2, 'wind', 15, 'kT', 0.95, 'CD', 1.2};
cases = {'30 m, 2 m/s, wind 15, thrust -5 %, CD 1.2', accept;
         '30 m, wind +25 m/s', {'x0', 30, 'wind', 25};
         '30 m, wind -25 m/s', {'x0', 30, 'wind', -25};
         '30 m, 2 m/s, no wind', {'x0', 30, 'vx0', 2}};
rows = {};
example = cell(1, 2);
for c = 1:size(cases, 1)
    for k = [0 1]
        p = setp(params(), [cases{c, 2}, {'kDob', k}]);
        s = simulate_landing(p, 120);
        rows(end+1, :) = {cases{c, 1}, k, s.x(end), s.vx(end), ...
            rad2deg(max(abs(s.theta))), all(touchdown_ok(p, s))}; %#ok<AGROW>
        if c == 1, example{k + 1} = s; end
    end
end
w.cases = cell2table(rows, 'VariableNames', {'Case', 'kDob', 'x_m', 'vx_mps', 'PeakTilt_deg', 'Success'});
disp(w.cases);

% Sweeps on the acceptance case: worst margin = smallest of
% 1 - |x|/req.x and 1 - |vx|/req.vx, the requirement closest to failing.
p0 = params();
kGrid = [0 0.25 0.5 0.75 1];  tauGrid = [0.05 0.2 0.5 1 2];
w.kDob = sweep('kDob', kGrid, accept, p0);
w.tauDob = sweep('tauDob', tauGrid, accept, p0);
disp(w.kDob);  disp(w.tauDob);

% Figure: the acceptance case, observer off and on
p = setp(params(), accept);
fig = figure('Visible', 'off', 'Theme', 'light', 'Position', [100 100 900 720]);
tl = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, 'Wind 15 m/s, thrust 5 % low, C_D = 1.2: LQR without and with the disturbance observer');
ax = gobjects(3, 1);
for k = 1:3, ax(k) = nexttile(tl); hold(ax(k), 'on'); grid(ax(k), 'on'); end
cols = lines(2);  lbl = {'observer off (k_{Dob} = 0)', 'observer on (k_{Dob} = 1)'};
for k = 1:2
    s = example{k};
    plot(ax(1), s.t, s.x, 'Color', cols(k, :), 'LineWidth', 1.4, 'DisplayName', lbl{k});
    plot(ax(2), s.t, s.vx, 'Color', cols(k, :), 'LineWidth', 1.4, 'HandleVisibility', 'off');
end
% disturbance seen by the lateral law: everything but a*sin(theta) with
% the commanded thrust, and its filtered estimate (observer on)
s = example{2};
[tu, dTrue, dHat] = disturbance(p, s);
plot(ax(3), tu, dTrue, 'k', 'LineWidth', 1.2, 'DisplayName', 'actual');
plot(ax(3), tu, dHat, '--', 'Color', cols(2, :), 'LineWidth', 1.4, 'DisplayName', 'estimate (filtered)');
yline(ax(1), [-p.req.x p.req.x], ':', 'HandleVisibility', 'off');
yline(ax(2), [-p.req.vx p.req.vx], ':', 'HandleVisibility', 'off');
ign = s.t(find(s.T > 0, 1));
for k = 1:2, xline(ax(k), ign, '--', 'ignition', 'HandleVisibility', 'off'); end
ylabel(ax(1), 'x [m]');  ylabel(ax(2), 'v_x [m/s]');  ylabel(ax(3), 'disturbance [m/s^2]');
xlabel(ax(3), 't [s]');  legend(ax(1), 'Location', 'northeast');  legend(ax(3), 'Location', 'northeast');
xlim(ax(3), [ign, s.t(end)]);
exportgraphics(fig, fullfile(figDir, 'lateral_wind.png'), 'Resolution', 150);
close(fig);
end

function tab = sweep(name, grid, accept, p0)
% Acceptance case flown for each value of p.(name).
rows = zeros(numel(grid), 4);
for i = 1:numel(grid)
    p = setp(p0, [accept, {name, grid(i)}]);
    s = simulate_landing(p, 120);
    margin = min(1 - abs(s.x(end))/p.req.x, 1 - abs(s.vx(end))/p.req.vx);
    rows(i, :) = [grid(i), s.x(end), s.vx(end), 100*margin];
end
tab = array2table(rows, 'VariableNames', {name, 'x_m', 'vx_mps', 'WorstMargin_pct'});
end

function [tu, dTrue, dHat] = disturbance(p, s)
% Lateral disturbance per unit mass as the observer defines it,
% d = ax - (Tcmd/m) sin(theta), rebuilt from the outputs (Tcmd = T/kT),
% and the same signal through the observer's filter 1/(tauDob s + 1).
tu = (s.t(1):0.01:s.t(end))';
r = @(v) interp1(s.t, v, tu);
z = r(s.z);  vx = r(s.vx);  vz = r(s.vz);  th = r(s.theta);  m = r(s.m);  T = r(s.T);  de = r(s.delta);
rho = p.rho0*exp(-z/p.H);  V = sqrt((vx - p.wind).^2 + vz.^2);
ax = (T.*sin(th + de) - 0.5*rho*p.CD*p.A.*V.*(vx - p.wind))./m;
dTrue = ax - (T/p.kT)./m.*sin(th);
dHat = lsim(tf(1, [p.tauDob 1]), dTrue, tu - tu(1));
on = T > 0;  dTrue(~on) = NaN;  dHat(~on) = NaN;
end

function ok = touchdown_ok(p, s)
ok = [s.t(end) < 120, abs(s.x(end)) < p.req.x, abs(s.vx(end)) < p.req.vx, ...
      abs(s.vz(end)) < p.req.vz, abs(s.theta(end)) < p.req.theta];
end

function p = setp(p, kv)
% Overwrite fields of p from a name/value list.
for j = 1:2:numel(kv), p.(kv{j}) = kv{j + 1}; end
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
