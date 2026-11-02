function [h, tb] = hoverslam_ignition(p, v0)
%HOVERSLAM_IGNITION Ignition altitude of a constant full-thrust landing burn.
%
%   [h, tb] = hoverslam_ignition(p, v0) returns the altitude h [m] at which
%   a vertical stage falling at speed v0 > 0 [m/s] must light its engine at
%   p.Tmax to reach zero velocity exactly at z = 0, and the burn time tb [s].
%   Drag is neglected and the mass falls linearly at Tmax/(Isp*g0).
%
%   With c = Isp*g0 and mdot = Tmax/c, the burn time solves the rocket
%   equation with gravity,
%       0 = -v0 - g*tb + c*ln(m0/(m0 - mdot*tb)),
%   and h is the distance covered during the burn,
%       h = v0*tb + g*tb^2/2 - c*(tb - (m(tb)/mdot)*ln(m0/m(tb))).

c    = p.Isp*p.g0;
mdot = p.Tmax/c;
dv   = @(t) -v0 - p.g*t + c*log(p.m0./(p.m0 - mdot*t));
tMax = (p.m0 - p.mdry)/mdot;        % burnout time
if dv(tMax) < 0
    error('hoverslam_ignition:propellant', ...
        'Not enough propellant to stop from %.1f m/s at full thrust.', v0);
end
tb = fzero(dv, [1e-6 tMax]);

mb = p.m0 - mdot*tb;
L  = log(p.m0/mb);
h  = v0*tb + 0.5*p.g*tb^2 - c*(tb - (mb/mdot)*L);
end
