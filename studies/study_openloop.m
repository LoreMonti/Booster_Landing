function summary = study_openloop(figDir)
%STUDY_OPENLOOP Why the landing burn needs guidance: two open-loop descents.
%
%   summary = study_openloop(figDir) simulates the stage from the start of
%   the terminal phase (p.z0, p.vz0) until touchdown with
%     - no burn: drag alone slows the stage towards terminal velocity;
%     - full thrust from t = 0: the burn starts far too early, the stage
%       stops in mid-air, climbs, runs out of propellant and falls.
%   Both end in a high-speed impact, which is the problem steps 2-3 solve.
%   Saves figures/openloop_descent.png and returns the touchdown values.

tEnd = 300;     % long enough for either case to reach the ground [s]

p = params();  p.T = 0;
noBurn = simulate_plant(p, tEnd);

p = params();  p.T = p.Tmax;
early = simulate_plant(p, tEnd);
kdot  = p.Tmax/(p.Isp*p.g0);
tBurnout = (p.m0 - p.mdry)/kdot;     % propellant above dry mass at Tmax [s]

runs  = {noBurn, early};
names = {'No burn', 'Full thrust from 5 km'};

fig = figure('Visible', 'off', 'Theme', 'light', 'Position', [100 100 900 700]);
tl = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, 'Open-loop descents from z_0 = 5 km, v_{z,0} = -280 m/s');

ax1 = nexttile(tl);  hold(ax1, 'on');
ax2 = nexttile(tl);  hold(ax2, 'on');
ax3 = nexttile(tl);  hold(ax3, 'on');
for i = 1:2
    s = runs{i};
    plot(ax1, s.t, s.z/1e3, 'LineWidth', 1.4, 'DisplayName', names{i});
    plot(ax2, s.t, s.vz,    'LineWidth', 1.4, 'DisplayName', names{i});
    plot(ax3, s.t, s.m/1e3, 'LineWidth', 1.4, 'DisplayName', names{i});
end
for ax = [ax1 ax2 ax3]
    xline(ax, tBurnout, '--', 'burnout', 'HandleVisibility', 'off', ...
        'LabelVerticalAlignment', 'bottom');
    grid(ax, 'on');
end
yline(ax3, p.mdry/1e3, ':', 'dry mass', 'HandleVisibility', 'off');
ylabel(ax1, 'altitude z [km]');
ylabel(ax2, 'vertical velocity v_z [m/s]');
ylabel(ax3, 'mass m [t]');
xlabel(ax3, 't [s]');
legend(ax1, 'Location', 'northeast');
exportgraphics(fig, fullfile(figDir, 'openloop_descent.png'), 'Resolution', 150);
close(fig);

summary = table(names', cellfun(@(s) s.t(end), runs)', cellfun(@(s) s.vz(end), runs)', ...
    cellfun(@(s) s.m(end), runs)', ...
    'VariableNames', {'Case', 'TouchdownTime_s', 'TouchdownVz_mps', 'FinalMass_kg'});
disp(summary);
end
