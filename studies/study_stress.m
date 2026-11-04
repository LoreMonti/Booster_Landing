function st = study_stress(figDir, scales, n)
%STUDY_STRESS Where the landing breaks: Monte Carlo with scaled dispersions.
%
%   st = study_stress(figDir) repeats the Monte Carlo of step 5 with every
%   1-sigma dispersion of params.m multiplied by 1, 1.5, 2, 2.5, 3 and 4,
%   300 landings each. A 100 % success rate at nominal dispersions says
%   nothing about the margin; growing them until landings fail shows how
%   far the design is from its limits and which error breaks it first.
%   st = study_stress(figDir, scales, n) sets the scale factors and the
%   number of landings per scale.
%
%   For every failed landing it records which requirement was violated
%   and which dispersion was the most extreme (largest number of sigmas).
%   Saves figures/stress_test.png and returns the table of results.

if nargin < 2, scales = [1 1.5 2 2.5 3 4]; end
if nargin < 3, n = 300; end
p0 = params();
reqs = {'touchdown', 'x', 'vx', 'vz', 'theta', 'prop'};
sig = struct('kT', p0.mc.kT, 'Isp', p0.mc.IspRel*p0.Isp, 'm0', p0.mc.m0, ...
    'CD', p0.mc.CDRel*p0.CD, 'wind', p0.mc.wind, 'x0', p0.mc.x0, 'vx0', p0.mc.vx0, ...
    'vz0', p0.mc.vz0, 'theta0', p0.mc.theta0);
nom = struct('kT', 1, 'Isp', p0.Isp, 'm0', p0.m0, 'CD', p0.CD, 'wind', 0, 'x0', 0, ...
    'vx0', 0, 'vz0', p0.vz0, 'theta0', 0);
factors = fieldnames(sig);

rows = cell(numel(scales), 1);
modeCount = zeros(numel(scales), numel(reqs));
driver = zeros(numel(scales), numel(factors));
for k = 1:numel(scales)
    tic;
    [d, r] = dispersed_landings(p0, n, scales(k), p0.mc.seed + k);
    nOk = sum(r.success);
    [~, lo, hi] = wilson(nOk, n);
    for j = 1:numel(reqs), modeCount(k, j) = sum(~r.ok.(reqs{j})); end
    % the most extreme draw of every failed landing, in its own sigmas
    z = zeros(n, numel(factors));
    for f = 1:numel(factors)
        z(:, f) = abs(d.(factors{f}) - nom.(factors{f}))/sig.(factors{f});
    end
    [~, top] = max(z(~r.success, :), [], 2);
    driver(k, :) = histcounts(top, 0.5:1:numel(factors) + 0.5);
    rows{k} = {scales(k), n, nOk, 100*nOk/n, 100*lo, 100*hi, prctile(abs(r.x), 99), ...
        prctile(abs(r.vz), 99), min(r.prop)};
    fprintf('scale %.1f: %d / %d succeed (%.0f s)\n', scales(k), nOk, n, toc);
end
st = cell2table(vertcat(rows{:}), 'VariableNames', {'Scale', 'Runs', 'Success', ...
    'SuccessPct', 'CI95lo', 'CI95hi', 'X99_m', 'Vz99_mps', 'MinPropellant_kg'});
disp(st);
disp(array2table(modeCount, 'VariableNames', reqs, 'RowNames', compose('x%.1f', scales)));
disp(array2table(driver, 'VariableNames', factors, 'RowNames', compose('x%.1f', scales)));
st = struct('summary', st, 'violations', modeCount, 'drivers', driver);

% Figure
fig = figure('Visible', 'off', 'Theme', 'light', 'Position', [100 100 1000 780]);
tl = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf('Stress test: every dispersion scaled by k, %d landings per k', n));
ax = nexttile(tl);  hold(ax, 'on');  grid(ax, 'on');
S = st.summary;
errorbar(ax, S.Scale, S.SuccessPct, S.SuccessPct - S.CI95lo, S.CI95hi - S.SuccessPct, 'o-', ...
    'LineWidth', 1.4, 'MarkerFaceColor', 'auto');
ylabel(ax, 'success [%]');  xlabel(ax, 'dispersion scale k');  ylim(ax, [0 105]);
title(ax, 'Success rate with 95 % confidence interval');
ax = nexttile(tl);
bar(ax, scales, modeCount, 'stacked');  grid(ax, 'on');
legend(ax, {'no touchdown', '|x|', '|v_x|', '|v_z|', '|\theta|', 'propellant'}, ...
    'Location', 'northwest', 'NumColumns', 3);
ylabel(ax, 'violations');  xlabel(ax, 'dispersion scale k');
title(ax, 'Requirement violated (a failed landing can violate several)');
ax = nexttile(tl);
hb = bar(ax, scales, driver, 'stacked');  grid(ax, 'on');
cmap = turbo(numel(factors));       % nine factors: the default palette has seven colours
for f = 1:numel(hb), hb(f).FaceColor = cmap(f, :); end
legend(ax, {'thrust', 'I_{sp}', 'm_0', 'C_D', 'wind', 'x_0', 'v_{x0}', 'v_{z0}', '\theta_0'}, ...
    'Location', 'northwest', 'NumColumns', 5);
ylabel(ax, 'failed landings');  xlabel(ax, 'dispersion scale k');
title(ax, 'Most extreme dispersion of each failed landing');
exportgraphics(fig, fullfile(figDir, 'stress_test.png'), 'Resolution', 150);
close(fig);
end

function [pHat, lo, hi] = wilson(k, n)
% Wilson score interval at 95 % for k successes in n trials.
z = 1.96;  pHat = k/n;
c = (pHat + z^2/(2*n))/(1 + z^2/n);
h = z*sqrt(pHat*(1 - pHat)/n + z^2/(4*n^2))/(1 + z^2/n);
lo = c - h;  hi = c + h;
end
