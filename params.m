function p = params()
%PARAMS Vehicle, environment and initial-state parameters for the landing models.
%
%   p = params() returns a struct read by every Simulink model of the
%   project. Blocks reference fields such as p.g or p.Isp instead of
%   literal numbers, so a study (e.g. the Monte Carlo of step 5) only has
%   to change this struct.
%
%   Vehicle: a Falcon 9-class first stage in its landing burn, one
%   Merlin 1D engine lit. It is representative, not a replica: values
%   marked [public] come from public sources, values marked [estimate]
%   are derived here (see the comments) and are dispersed in step 5.
%
%   Sources:
%     [1] Wikipedia, "SpaceX Merlin" (Merlin 1D thrust, throttle, Isp).
%     [2] Wikipedia, "Falcon 9 Full Thrust" (diameter, length).
%     [3] Spaceflight101, "Falcon 9 v1.1 & F9R" (inert mass with legs).

% Environment
p.g0   = 9.80665;   % standard gravity, used in the rocket equation [m/s^2]
p.g    = 9.80665;   % local gravity at the landing site [m/s^2]
p.rho0 = 1.225;     % sea-level air density [kg/m^3]
p.H    = 8500;      % scale height of the exponential atmosphere [m]

% Propulsion: one Merlin 1D at sea level
p.Tmax = 845e3;         % maximum thrust [N]                        [public, 1]
p.Tmin = 0.40*p.Tmax;   % minimum throttle 40 % (sources: 40-57 %)  [public, 1]
p.Isp  = 282;           % specific impulse [s]                      [public, 1]
p.T    = p.Tmax;        % constant thrust command of the open-loop step 1 models [N]

% Mass
p.mdry = 25.6e3;    % inert mass with legs and grid fins [kg]      [public, 3]
p.m0   = 32e3;      % mass at landing-burn ignition [kg]           [estimate]
                    % rocket equation with dv ~ 600 m/s (280 m/s braking
                    % plus ~30 s of gravity loss): m0/mf = exp(600/2766) ~ 1.24

% Geometry, inertia and aerodynamics
p.L  = 42;              % stage length [m]                          [public, 2]
p.d  = 3.66;            % stage diameter [m]                        [public, 2]
p.A  = pi*p.d^2/4;      % reference area for drag [m^2]
p.CD = 0.8;             % axial drag coefficient, base first        [estimate]
p.l  = 16;              % gimbal-to-centre-of-mass distance [m]     [estimate]
                        % 6 t of engines at 1 m + 19.6 t uniform over 42 m
p.I  = 4.8e6;           % pitch moment of inertia [kg m^2]          [estimate]
                        % same mass split: rod + parallel-axis terms

% Actuators (assumed, used from step 2)
p.deltaMax  = deg2rad(5);   % gimbal limit [rad]
p.deltaRate = deg2rad(10);  % gimbal rate limit [rad/s]
p.tauAct    = 0.1;          % gimbal servo time constant (first-order lag) [s]

% Attitude loop (step 2): PD on the commanded pitch acceleration, then
% delta = -thetaddot_cmd * I/(T*l), so the closed loop
% s^2 + Kd*s + Kp is the same at every throttle setting.
% wn = 1 rad/s keeps a 5 deg error within the authority at Tmin.
p.wnAtt   = 1;                      % attitude natural frequency [rad/s]
p.zetaAtt = 0.7;                    % attitude damping ratio [-]
p.KpAtt   = p.wnAtt^2;              % [1/s^2]
p.KdAtt   = 2*p.zetaAtt*p.wnAtt;    % [1/s]
p.thetaRef = 0;                     % pitch reference of the step 2 models [rad]

% Vertical guidance (step 3): constant-deceleration law in feedback.
% aReq = (vz^2 - vTouch^2)/(2*z) is the constant deceleration that brings
% the stage to vTouch at z = 0. The engine lights when aReq reaches
% aStar = eta*(Tmax/m0 - g) and then commands T = m*(g + aReq), clipped
% to [Tmin, Tmax]. eta < 1 keeps throttle margin for dispersions: with
% eta = 1 a 3 % thrust or mass error ends in a ~75 m/s impact.
p.eta    = 0.7;                             % planned fraction of the maximum deceleration [-]
p.aStar  = p.eta*(p.Tmax/p.m0 - p.g);       % ignition deceleration threshold [m/s^2]
p.aOff   = 0.1*p.aStar;                     % engine-off threshold of the relay [m/s^2]
p.vTouch = 1;                               % target touchdown speed [m/s]
p.zMin   = 0.05;                            % altitude floor in aReq, avoids 0/0 at touchdown [m]
p.kT     = 1;                               % actual/commanded thrust (1 = no thrust error) [-]

% Touchdown requirements (PROJECT_BRIEF.md, section 2)
p.req.vz    = 2;            % |vertical speed| [m/s]
p.req.vx    = 1;            % |horizontal speed| [m/s]
p.req.theta = deg2rad(5);   % |tilt| [rad]
p.req.x     = 10;           % |distance from the pad| [m]

% Initial state at the start of the terminal phase.
% theta is measured from the vertical, positive when the vehicle leans
% towards +x; delta > 0 turns the thrust towards +x.
p.x0     = 0;       % horizontal position, pad at x = 0 [m]
p.z0     = 5000;    % altitude above the pad [m]
p.vx0    = 0;       % horizontal velocity [m/s]
p.vz0    = -280;    % vertical velocity, positive up [m/s]; terminal
                    % velocity at 5 km is ~320 m/s for these values
p.theta0 = 0;       % pitch angle [rad]
p.q0     = 0;       % pitch rate [rad/s]
p.delta  = 0;       % constant gimbal command of the open-loop step 1 models [rad]

end
