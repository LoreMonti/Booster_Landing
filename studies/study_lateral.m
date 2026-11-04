function summary = study_lateral(figDir)
%STUDY_LATERAL Cascaded PD against LQR for the lateral landing error (step 4).
%
%   summary = study_lateral(figDir) flies models/landing.slx with the two
%   lateral gain sets of params.m, which share the attitude controller,
%   the gimbal and the guidance and differ only in the four gains K of
%       thetaRef = -(K1 x + K2 vx)/a + K3 theta + K4 q.
%   Scenarios: four lateral errors at the start of the terminal phase,
%   each nominal and with the combined thrust, drag and Isp error. It
%   saves figures/lateral_comparison.png (x, theta and delta for the
%   30 m / 2 m/s case) and prints and returns the comparison table.

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
