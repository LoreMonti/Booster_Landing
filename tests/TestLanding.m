classdef TestLanding < matlab.unittest.TestCase
%TESTLANDING Checks the vertical guidance (step 3) and models/landing.slx.
%
%   hoverslamIgnition  the closed-form ignition altitude of a full-thrust
%                      burn (utils/hoverslam_ignition.m) stops the plant
%                      exactly at the ground, drag off.
%   landing            the guided landing meets every touchdown requirement
%                      (p.req in params.m), nominally and with thrust,
%                      drag and Isp errors the guidance does not know about,
%                      alone and combined;
%                      the thrust stays in [Tmin, Tmax] while the engine
%                      runs and the engine lights exactly once.
%   nominalEfficiency  the nominal burn keeps the planned constant
%                      deceleration instead of being over-braked by drag.
%
%   Model interface expected by the tests:
%     - parameters read from the struct p (see params.m);
%     - plant_3dof referenced through a Model block, attitude_ctrl
%       through a Subsystem Reference;
%     - root Outports 1-7 = x, z, vx, vz, theta, q, m, 8 = delta and
%       9 = thrust applied to the plant.
%
%   Run from the repository root:  runtests('tests')

    properties (Constant)
        Model = 'landing'
    end

    properties (TestParameter)
        % Errors unknown to the guidance, which keeps assuming kT = 1,
        % CDhat and the nominal Isp (reference model: touchdown at
        % 1.15-1.38 m/s, 818-2263 kg of propellant left). thrustLow is
        % the case that fails without the integral term (2.4 m/s).
        dispersion = struct( ...
            'nominal',    {{}}, ...
            'thrustLow',  {{'kT', 0.95}}, ...
            'thrustLow10',{{'kT', 0.90}}, ...
            'thrustHigh', {{'kT', 1.05}}, ...
            'dragLow',    {{'CD', 0.4}}, ...
            'dragHigh',   {{'CD', 1.2}}, ...
            'ispLow',     {{'Isp', 0.95*282}}, ...
            'combined',   {{'kT', 0.95, 'CD', 0.4, 'Isp', 0.95*282}})
    end

    methods (TestClassSetup)
        function addPaths(tc)
            root = fileparts(fileparts(mfilename('fullpath')));
            tc.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            tc.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, 'models')));
            tc.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, 'utils')));
        end
    end

    methods (Test)
        function hoverslamIgnition(tc)
            % Light at Tmax at the closed-form altitude h, no drag: after
            % the burn time tb the stage must be at rest on the ground
            % (reference: z = 4e-10 m, vz = 1.3e-4 m/s).
            p = params();
            [h, tb] = hoverslam_ignition(p, 280);
            p.CD = 0;  p.z0 = h;  p.vz0 = -280;  p.T = p.Tmax;
            s = simulate_plant(p, tb, 1e-10);

            tc.verifyEqual(s.t(end), tb, 'AbsTol', 1e-6);
            tc.verifyEqual(s.z(end),  0, 'AbsTol', 1e-3);
            tc.verifyEqual(s.vz(end), 0, 'AbsTol', 1e-2);
        end

        function landing(tc, dispersion)
            p = params();
            for j = 1:2:numel(dispersion)
                p.(dispersion{j}) = dispersion{j + 1};
            end
            s = tc.simulate(p, 120);

            % touched down before the time limit
            tc.assertLessThan(s.t(end), 120, 'No touchdown within 120 s.');
            tc.verifyLessThan(abs(s.vz(end)),    p.req.vz);
            tc.verifyLessThan(abs(s.vx(end)),    p.req.vx);
            tc.verifyLessThan(abs(s.theta(end)), p.req.theta);
            tc.verifyLessThan(abs(s.x(end)),     p.req.x);
            tc.verifyGreaterThan(s.m(end), p.mdry);

            % throttle limits while the engine runs, and a single ignition
            on = s.T > 0;
            tc.verifyGreaterThanOrEqual(min(s.T(on)), p.kT*p.Tmin*(1 - 1e-9));
            tc.verifyLessThanOrEqual(max(s.T(on)),    p.kT*p.Tmax*(1 + 1e-9));
            tc.verifyEqual(sum(diff(on) > 0), 1, 'The engine must light exactly once.');
        end
    end

    methods (Test)
        function nominalEfficiency(tc)
            % With the drag feedforward the nominal burn follows the
            % planned constant deceleration: it lasts about
            % (|vz| at ignition)/aStar = 25 s and leaves 1686 kg on the
            % reference model. Without it the drag over-brakes the stage:
            % 34 s and 635 kg.
            p = params();
            s = tc.simulate(p, 120);
            on = s.T > 0;
            burn = s.t(end) - s.t(find(on, 1));
            tc.verifyLessThan(burn, 27);
            tc.verifyGreaterThan(s.m(end) - p.mdry, 1500);
        end
    end

    methods (Access = private)
        function s = simulate(tc, p, tEnd)
            in = Simulink.SimulationInput(tc.Model);
            in = in.setVariable('p', p);
            in = in.setModelParameter('Solver', 'ode45', ...
                'RelTol', '1e-8', 'AbsTol', '1e-8', 'StopTime', num2str(tEnd), ...
                'SaveOutput', 'on', 'OutputSaveName', 'yout', 'SaveFormat', 'Dataset');
            out = sim(in);
            tc.assertEmpty(out.ErrorMessage);

            names = {'x', 'z', 'vx', 'vz', 'theta', 'q', 'm', 'delta', 'T'};
            for i = 1:numel(names)
                s.(names{i}) = out.yout{i}.Values.Data;
            end
            s.t = out.yout{1}.Values.Time;
        end
    end
end
