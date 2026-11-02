function s = simulate_plant(p, tEnd, relTol)
%SIMULATE_PLANT Runs models/plant_3dof.slx with parameters p and returns its states.
%
%   s = simulate_plant(p, tEnd) simulates up to tEnd seconds, or until
%   touchdown (z <= 0) if that comes first. The plant inputs (root
%   Inports 1-2) are held at the constant commands p.T and p.delta.
%   s has fields t, x, z, vx, vz, theta, q, m (column vectors, SI units, rad).
%
%   s = simulate_plant(p, tEnd, relTol) sets the ode45 relative and
%   absolute tolerance (default 1e-8).

if nargin < 3, relTol = 1e-8; end

in = Simulink.SimulationInput('plant_3dof');
in = in.setVariable('p', p);
in = in.setExternalInput([0 p.T p.delta; tEnd p.T p.delta]);
in = in.setModelParameter('Solver', 'ode45', ...
    'RelTol', num2str(relTol), 'AbsTol', num2str(relTol), ...
    'StopTime', num2str(tEnd), 'SaveOutput', 'on', ...
    'OutputSaveName', 'yout', 'SaveFormat', 'Dataset');
out = sim(in);
if ~isempty(out.ErrorMessage)
    error('simulate_plant:failed', '%s', out.ErrorMessage);
end

% Root Outports 1-7, in the order fixed by the model interface
names = {'x', 'z', 'vx', 'vz', 'theta', 'q', 'm'};
for i = 1:numel(names)
    s.(names{i}) = out.yout{i}.Values.Data;
end
s.t = out.yout{1}.Values.Time;
end
