function mc = study_montecarlo(figDir, n)
%STUDY_MONTECARLO Dispersion analysis of the guided landing (step 5).
%
%   mc = study_montecarlo(figDir) flies models/landing.slx p.mc.n times
%   with the Gaussian dispersions of params.m (thrust, Isp, ignition
%   mass, drag coefficient, wind and initial state), drawn with a fixed
%   seed. The guidance and controllers keep the nominal values. A
%   landing succeeds if it touches down within 120 s, meets every
%   touchdown requirement and still has propellant.
%   mc = study_montecarlo(figDir, n) flies n landings instead.
%
%   Saves
%     figures/montecarlo_touchdown.png    touchdown scatter with the 99 %
%                                         covariance ellipse and histograms;
%     figures/montecarlo_convergence.png  success rate and spread against
%                                         the number of runs;
%     results/montecarlo.mat              every draw and outcome (not tracked);
%   prints the summary and the failed cases, and returns a struct.

p0 = params();
if nargin < 2, n = p0.mc.n; end

tic;
[d, r] = dispersed_landings(p0, n, 1, p0.mc.seed);
fprintf('%d landings in %.0f s\n', n, toc);
req = p0.req;
ok = r.ok;

% Summary
[pHat, lo, hi] = wilson(sum(r.success), n);
mc.n = n;  mc.successRate = pHat;  mc.ci95 = [lo hi];
mc.x99 = prctile(abs(r.x), 99);  mc.vz99 = prctile(abs(r.vz), 99);
mc.propMin = min(r.prop);  mc.prop1 = prctile(r.prop, 1);
fprintf('\nSuccess: %d / %d = %.2f %% (95 %% CI %.2f-%.2f %%)\n', sum(r.success), n, 100*pHat, 100*lo, 100*hi);
fprintf('99th percentile: |x| %.2f m, |vz| %.2f m/s, |vx| %.2f m/s, |theta| %.2f deg\n', ...
    mc.x99, mc.vz99, prctile(abs(r.vx), 99), rad2deg(prctile(abs(r.theta), 99)));
fprintf('Propellant left: min %.0f kg, 1st percentile %.0f kg, median %.0f kg\n', ...
    mc.propMin, mc.prop1, median(r.prop));
fprintf('Peak tilt max %.1f deg, peak gimbal max %.2f deg, ignitions per run %s\n', ...
    rad2deg(max(r.tilt)), rad2deg(max(r.gimbal)), mat2str(unique(r.ign)'));
names = fieldnames(ok);
for k = 1:numel(names)
    fprintf('  requirement %-9s violated in %d runs\n', names{k}, sum(~ok.(names{k})));
end
bad = find(~r.success);
if ~isempty(bad)
    fprintf('\nFailed runs:\n');
    disp(table(bad, d.kT(bad), d.Isp(bad), d.m0(bad), d.CD(bad), d.wind(bad), d.x0(bad), ...
        d.vx0(bad), d.vz0(bad), rad2deg(d.theta0(bad)), r.x(bad), r.vx(bad), r.vz(bad), r.prop(bad), ...
        'VariableNames', {'run', 'kT', 'Isp', 'm0', 'CD', 'wind', 'x0', 'vx0', 'vz0', 'theta0_deg', ...
        'x_td', 'vx_td', 'vz_td', 'prop_kg'}));
end
mc.draws = d;  mc.result = r;

root = fileparts(fileparts(mfilename('fullpath')));
if ~exist(fullfile(root, 'results'), 'dir'), mkdir(fullfile(root, 'results')); end
save(fullfile(root, 'results', 'montecarlo.mat'), 'mc');

% Figure 1: touchdown dispersion
fig = figure('Visible', 'off', 'Theme', 'light', 'Position', [100 100 1000 760]);
tl = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf('Monte Carlo, %d landings: %.1f %% success (95 %% CI %.1f-%.1f %%)', ...
    n, 100*pHat, 100*lo, 100*hi));
ax = nexttile(tl);  hold(ax, 'on');  grid(ax, 'on');
plot(ax, r.x(r.success), r.vz(r.success), '.', 'Color', [0 0.45 0.74], 'DisplayName', 'success');
plot(ax, r.x(~r.success), r.vz(~r.success), 'rx', 'MarkerSize', 8, 'LineWidth', 1.2, 'DisplayName', 'failure');
[ex, ey] = ellipse99([r.x r.vz]);
plot(ax, ex, ey, 'k-', 'LineWidth', 1.4, 'DisplayName', '99 % ellipse');
rectangle(ax, 'Position', [-req.x -req.vz 2*req.x req.vz], 'EdgeColor', [0.5 0.5 0.5], 'LineStyle', '--');
xlabel(ax, 'x at touchdown [m]');  ylabel(ax, 'v_z at touchdown [m/s]');
title(ax, 'Touchdown state (dashed: requirement box)');  legend(ax, 'Location', 'southeast');
ax = nexttile(tl);
histogram(ax, r.x, 40);  grid(ax, 'on');
xline(ax, prctile(r.x, [0.5 99.5]), 'k--', {'0.5 %', '99.5 %'});
xlabel(ax, 'x at touchdown [m]');  title(ax, 'Lateral miss');
ax = nexttile(tl);
histogram(ax, r.vx, 40);  grid(ax, 'on');
xline(ax, [-req.vx req.vx], 'r--', 'requirement');
xlabel(ax, 'v_x at touchdown [m/s]');  title(ax, 'Lateral speed');
ax = nexttile(tl);
histogram(ax, r.prop, 40);  grid(ax, 'on');
xline(ax, 0, 'r--', 'empty');
xlabel(ax, 'propellant left [kg]');  title(ax, 'Propellant margin');
exportgraphics(fig, fullfile(figDir, 'montecarlo_touchdown.png'), 'Resolution', 150);
close(fig);

% Figure 2: convergence with the number of runs
N = (10:n)';
cs = cumsum(r.success);  rate = cs(N)./N;
[~, clo, chi] = arrayfun(@(k) wilson(cs(k), k), N);
sx = arrayfun(@(k) std(r.x(1:k)), N);
fig = figure('Visible', 'off', 'Theme', 'light', 'Position', [100 100 900 560]);
tl = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, 'Convergence of the Monte Carlo statistics');
ax = nexttile(tl);  hold(ax, 'on');  grid(ax, 'on');
fill(ax, [N; flipud(N)], 100*[clo; flipud(chi)], [0.8 0.85 0.95], 'EdgeColor', 'none', ...
    'DisplayName', '95 % confidence interval');
plot(ax, N, 100*rate, 'LineWidth', 1.3, 'DisplayName', 'success rate');
ylabel(ax, 'success [%]');  legend(ax, 'Location', 'southeast');  set(ax, 'XScale', 'log');
ax = nexttile(tl);
plot(ax, N, sx, 'LineWidth', 1.3);  grid(ax, 'on');  set(ax, 'XScale', 'log');
ylabel(ax, '\sigma of x at touchdown [m]');  xlabel(ax, 'number of runs');
exportgraphics(fig, fullfile(figDir, 'montecarlo_convergence.png'), 'Resolution', 150);
close(fig);
end

function [pHat, lo, hi] = wilson(k, n)
% Wilson score interval at 95 % for k successes in n trials.
z = 1.96;  pHat = k/n;
c = (pHat + z^2/(2*n))/(1 + z^2/n);
h = z*sqrt(pHat*(1 - pHat)/n + z^2/(4*n^2))/(1 + z^2/n);
lo = c - h;  hi = c + h;
end

function [ex, ey] = ellipse99(X)
% 99 % covariance ellipse of the rows of X (chi-square, 2 dof).
mu = mean(X, 1);  [V, D] = eig(cov(X));
a = linspace(0, 2*pi, 200);
E = V*sqrt(D)*sqrt(chi2inv(0.99, 2))*[cos(a); sin(a)];
ex = mu(1) + E(1, :);  ey = mu(2) + E(2, :);
end
