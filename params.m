function p = params()
%PARAMS Vehicle, environment and initial-state parameters for the landing models.
%
%   p = params() returns a struct read by every Simulink model of the
%   project. Blocks reference fields such as p.g or p.Isp instead of
%   literal numbers, so a study (e.g. the Monte Carlo of step 5) only has
%   to change this struct.
%
%   Values marked PROVISIONAL are round numbers for building and testing
%   the vertical channel (step 1a). They will be replaced by values taken
%   from public data on 10-30 t first stages (step 1b).

% Environment
p.g0 = 9.80665;     % standard gravity, used in the rocket equation [m/s^2]
p.g  = 9.80665;     % local gravity at the landing site [m/s^2]

% Propulsion (PROVISIONAL)
p.Isp = 280;        % sea-level specific impulse [s]
p.T   = 400e3;      % constant thrust used by the step 1a models [N]

% Mass (PROVISIONAL)
p.m0   = 25e3;      % mass at ignition of the landing burn [kg]
p.mdry = 20e3;      % dry mass: the burn must end above this [kg]

% Initial state of the vertical channel (PROVISIONAL)
p.z0  = 1000;       % altitude above the pad [m]
p.vz0 = -100;       % vertical velocity, positive up [m/s]

end
