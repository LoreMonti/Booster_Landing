function [d, r] = dispersed_landings(p0, n, scale, seed)
%DISPERSED_LANDINGS Flies models/landing.slx n times with random dispersions.
%
%   [d, r] = dispersed_landings(p0, n, scale, seed) draws n samples of the
%   Gaussian dispersions p0.mc (thrust, Isp, ignition mass, drag
%   coefficient, wind, initial state), each 1-sigma value multiplied by
%   scale, with random seed seed, and flies one landing per sample. The
%   guidance and controllers keep the nominal values of p0.
%
%   d holds the draws (one column vector per dispersed field of p).
%   r holds the outcomes: touchdown state x, vx, vz, theta, propellant
%   left prop, touchdown time t, peak tilt and gimbal, number of
%   ignitions ign, one logical vector per requirement in r.ok and the
%   overall r.success.

rng(seed);
s = scale;
d.kT     = 1 + s*p0.mc.kT*randn(n, 1);
d.Isp    = p0.Isp*(1 + s*p0.mc.IspRel*randn(n, 1));
d.m0     = p0.m0 + s*p0.mc.m0*randn(n, 1);
d.CD     = max(p0.CD*(1 + s*p0.mc.CDRel*randn(n, 1)), 0.1);
d.wind   = s*p0.mc.wind*randn(n, 1);
d.x0     = s*p0.mc.x0*randn(n, 1);
d.vx0    = s*p0.mc.vx0*randn(n, 1);
d.vz0    = p0.vz0 + s*p0.mc.vz0*randn(n, 1);
d.theta0 = s*p0.mc.theta0*randn(n, 1);
fields = fieldnames(d);

in(n) = Simulink.SimulationInput('landing');
for i = 1:n
    p = p0;
    for f = 1:numel(fields), p.(fields{f}) = d.(fields{f})(i); end
    in(i) = Simulink.SimulationInput('landing');
    in(i) = in(i).setVariable('p', p);
    in(i) = in(i).setModelParameter('Solver', 'ode45', 'RelTol', '1e-6', 'AbsTol', '1e-6', ...
        'StopTime', '120', 'SaveOutput', 'on', 'OutputSaveName', 'yout', 'SaveFormat', 'Dataset');
end
out = sim(in, 'ShowProgress', 'off', 'ShowSimulationManager', 'off');

[r.x, r.vx, r.vz, r.theta, r.prop, r.t, r.tilt, r.gimbal, r.ign] = deal(nan(n, 1));
for i = 1:n
    y = out(i).yout;
    if isempty(y), continue; end
    v = @(k) y{k}.Values.Data;
    x = v(1); vx = v(3); vz = v(4); th = v(5); m = v(7); de = v(8); T = v(9);
    r.x(i) = x(end);  r.vx(i) = vx(end);  r.vz(i) = vz(end);  r.theta(i) = th(end);
    r.prop(i) = m(end) - p0.mdry;  r.t(i) = y{1}.Values.Time(end);
    r.tilt(i) = max(abs(th));  r.gimbal(i) = max(abs(de));  r.ign(i) = sum(diff(T > 0) > 0);
end
req = p0.req;
r.ok.touchdown = r.t < 120;
r.ok.x     = abs(r.x) < req.x;
r.ok.vx    = abs(r.vx) < req.vx;
r.ok.vz    = abs(r.vz) < req.vz;
r.ok.theta = abs(r.theta) < req.theta;
r.ok.prop  = r.prop > 0;
r.success  = r.ok.touchdown & r.ok.x & r.ok.vx & r.ok.vz & r.ok.theta & r.ok.prop;
end
