function s = study_animation(figDir)
%STUDY_ANIMATION Animated GIF of one guided landing (extension 3).
%
%   s = study_animation(figDir) flies models/landing.slx from 30 m off the
%   pad, drifting away at 2 m/s, in a 10 m/s wind, with every controller
%   of params.m (guidance, attitude loop, LQR and disturbance observer),
%   and writes figures/landing.gif at twice real time. It returns the
%   simulated outputs in s.
%
%   The descent is 5 km tall and less than 100 m wide, so one picture at
%   true scale would hide both the lateral manoeuvre and the tilt:
%     left   the whole descent, z against x, with x stretched;
%     right  a 90 m window that follows the stage, at true scale: the
%            stage tilted by theta, the plume along theta + delta with a
%            length proportional to the thrust, the legs once below
%            500 m, the wind, and the pad with the 10 m requirement.

p = params();
p.x0 = 30;  p.vx0 = 2;  p.wind = 10;

in = Simulink.SimulationInput('landing');
in = in.setVariable('p', p);
in = in.setModelParameter('Solver', 'ode45', 'RelTol', '1e-8', 'AbsTol', '1e-8', ...
    'StopTime', '120', 'SaveOutput', 'on', 'OutputSaveName', 'yout', 'SaveFormat', 'Dataset');
out = sim(in);
names = {'x', 'z', 'vx', 'vz', 'theta', 'q', 'm', 'delta', 'T'};
for i = 1:numel(names)
    s.(names{i}) = out.yout{i}.Values.Data;
end
s.t = out.yout{1}.Values.Time;
fprintf('Animated landing: touchdown at t = %.1f s, x = %.2f m, vx = %.2f m/s, vz = %.2f m/s\n', ...
    s.t(end), s.x(end), s.vx(end), s.vz(end));

% Frames: twice real time at 15 frames per second, last frame held.
speed = 2;  fps = 15;
tf = [(0:speed/fps:s.t(end))'; s.t(end)];
f = @(v) interp1(s.t, v, tf);
x = f(s.x);  z = f(s.z);  vx = f(s.vx);  vz = f(s.vz);
th = f(s.theta);  de = f(s.delta);  T = f(s.T);  m = f(s.m);
ig = s.t(find(s.T > 0, 1));
xIg = interp1(s.t, s.x, ig);  zIg = interp1(s.t, s.z, ig);

fig = figure('Visible', 'off', 'Theme', 'light', 'Color', 'w', 'Position', [100 100 960 540]);
tl = tiledlayout(fig, 1, 5, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf(['Guided landing from 5 km: 30 m off the pad, drifting at 2 m/s, ' ...
    'wind %g m/s (%g\\times real time)'], p.wind, speed));

% Left: the whole descent
axL = nexttile(tl, [1 2]);  hold(axL, 'on');  grid(axL, 'on');
plot(axL, s.x, s.z, 'Color', [0.8 0.8 0.8], 'LineWidth', 1);
plot(axL, [-p.req.x p.req.x], [0 0], 'k', 'LineWidth', 4);
plot(axL, xIg, zIg, 'p', 'Color', [0.85 0.33 0.1], 'MarkerFaceColor', [0.85 0.33 0.1], 'MarkerSize', 10);
text(axL, xIg, zIg, '  ignition', 'Color', [0.85 0.33 0.1], 'VerticalAlignment', 'bottom');
trail = plot(axL, NaN, NaN, 'Color', [0 0.45 0.74], 'LineWidth', 1.8);
dot = plot(axL, NaN, NaN, 'o', 'Color', [0 0.45 0.74], 'MarkerFaceColor', [0 0.45 0.74]);
xr = [min(s.x), max(s.x)];  xr = xr + [-0.15 0.15]*diff(xr);
xlim(axL, [min(xr(1), -15), max(xr(2), 15)]);  ylim(axL, [-100, 1.04*p.z0]);
xlabel(axL, 'x [m]  (horizontal scale stretched)');  ylabel(axL, 'altitude z [m]');

% Right: true-scale window following the stage
axR = nexttile(tl, [1 3]);  hold(axR, 'on');  box(axR, 'on');
axis(axR, 'equal');  set(axR, 'XGrid', 'on', 'YGrid', 'on');
half = 45;                                  % half-width of the window [m]
patch(axR, [-1e3 1e3 1e3 -1e3], [-50 -50 0 0], [0.85 0.82 0.75], 'EdgeColor', 'none');
plot(axR, [-p.req.x p.req.x], [0 0], 'k', 'LineWidth', 5);
text(axR, 0, -4, 'pad (\pm10 m requirement)', 'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'top', 'FontSize', 9);
plume = patch(axR, NaN, NaN, [1 0.6 0.1], 'EdgeColor', [0.9 0.35 0], 'FaceAlpha', 0.85);
body  = patch(axR, NaN, NaN, [0.95 0.95 0.95], 'EdgeColor', [0.2 0.2 0.2], 'LineWidth', 1.2);
legs  = plot(axR, NaN, NaN, 'Color', [0.2 0.2 0.2], 'LineWidth', 2);
windA = quiver(axR, NaN, NaN, NaN, NaN, 0, 'Color', [0.3 0.6 0.9], 'LineWidth', 2, 'MaxHeadSize', 0.8);
windT = text(axR, NaN, NaN, sprintf('wind %g m/s', p.wind), 'Color', [0.3 0.6 0.9], 'FontSize', 9);
telem = text(axR, NaN, NaN, '', 'FontName', 'FixedWidth', 'FontSize', 9, 'VerticalAlignment', 'top', ...
    'BackgroundColor', [1 1 1 0.8]);
xlabel(axR, 'x [m]  (true scale)');  ylabel(axR, 'z [m]');

L = p.L;  r = p.d/2;  lPlume = 30;          % stage length, radius, plume at Tmax [m]
frames = cell(numel(tf), 1);
for k = 1:numel(tf)
    b = [x(k), max(z(k), 0)];               % base of the stage (engine)
    ax = [sin(th(k)), cos(th(k))];          % body axis, towards the nose
    nrm = [ax(2), -ax(1)];                  % normal to the body axis
    c = [b + r*nrm; b + r*nrm + L*ax; b + L*ax + 0.3*r*nrm; b + L*ax - 0.3*r*nrm; ...
         b + L*ax - r*nrm; b - r*nrm];      % body outline with a blunt nose
    set(body, 'XData', c(:, 1), 'YData', c(:, 2));

    lit = T(k) > 0 && k < numel(tf);        % engine cut at touchdown
    if lit
        u = -[sin(th(k) + de(k)), cos(th(k) + de(k))];     % plume direction
        w = [u(2), -u(1)];
        len = min(lPlume*T(k)/p.Tmax, b(2)/max(-u(2), 0.1));   % stops at the ground
        pl = [b + 0.8*r*w; b + len*u; b - 0.8*r*w];
        set(plume, 'XData', pl(:, 1), 'YData', pl(:, 2));
    else
        set(plume, 'XData', NaN, 'YData', NaN);
    end

    if z(k) < 500                           % legs deployed
        lg = [b + r*nrm + 6*ax; b + 2.5*r*nrm; NaN NaN; b - r*nrm + 6*ax; b - 2.5*r*nrm];
        set(legs, 'XData', lg(:, 1), 'YData', lg(:, 2));
    else
        set(legs, 'XData', NaN, 'YData', NaN);
    end

    lo = max(z(k) - 25, -12);               % keep the ground in view at the end
    xlim(axR, x(k) + [-half half]);  ylim(axR, [lo, lo + 2*half]);
    a0 = [x(k) - half + 4, lo + 2*half - 26];
    set(windA, 'XData', a0(1), 'YData', a0(2), 'UData', 1.2*p.wind, 'VData', 0);
    set(windT, 'Position', [a0(1), a0(2) - 3, 0]);
    if lit
        eng = sprintf('thrust %3.0f kN', T(k)/1e3);
    else
        eng = 'engine off     ';
    end
    zero = @(v, d) round(v, d) + 0;         % no "-0" in the telemetry
    set(telem, 'Position', [x(k) - half + 2, lo + 2*half - 2, 0], 'String', sprintf( ...
        't  %5.1f s    z  %6.0f m\nvz %6.1f m/s  vx %5.2f m/s\ntilt %5.1f deg  %s\npropellant %5.0f kg', ...
        tf(k), zero(max(z(k), 0), 0), vz(k), zero(vx(k), 2), zero(rad2deg(th(k)), 1), eng, m(k) - p.mdry));

    set(trail, 'XData', [s.x(s.t <= tf(k)); x(k)], 'YData', [s.z(s.t <= tf(k)); z(k)]);
    set(dot, 'XData', x(k), 'YData', z(k));
    frames{k} = print(fig, '-RGBImage', '-r80');
end
close(fig);

% One colour map for every frame keeps the GIF small and free of flicker.
[~, map] = rgb2ind(cat(1, frames{[1, round(end/2), end]}), 48, 'nodither');
gif = fullfile(figDir, 'landing.gif');
for k = 1:numel(frames)
    im = rgb2ind(frames{k}, map, 'nodither');
    if k == 1
        imwrite(im, map, gif, 'gif', 'LoopCount', Inf, 'DelayTime', 1/fps);
    elseif k < numel(frames)
        imwrite(im, map, gif, 'gif', 'WriteMode', 'append', 'DelayTime', 1/fps);
    else
        imwrite(im, map, gif, 'gif', 'WriteMode', 'append', 'DelayTime', 4);
    end
end
d = dir(gif);
fprintf('%s: %d frames, %.1f MB\n', gif, numel(frames), d.bytes/1e6);
end
