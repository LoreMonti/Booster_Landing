classdef TestFailures < matlab.unittest.TestCase
%TESTFAILURES Checks the failures injected into models/landing.slx (extension 4).
%
%   thrustLossWithinMargin  15 % of the thrust lost 10 s into the burn:
%                           the guidance, which is not told, still meets
%                           every touchdown requirement.
%   thrustLossBeyondMargin  30 % lost at the same time: more than the
%                           throttle margin, so the stage crashes; the
%                           thrust after the failure never exceeds
%                           70 % of Tmax, which pins the injection itself.
%   altimeterReadsHigh      +1 m bias, inside the terminal gate: the stage
%                           meets the ground still at
%                           sqrt(vTouch^2 + 2*aGate*1), 3.3 m/s.
%   gateToleratesBias       +0.25 m bias: within the 0.30 m the gate
%                           tolerates, the touchdown meets the requirement
%                           (2.6 m/s without the gate).
%   altimeterReadsLow       -10 m bias: the stage stops above the ground
%                           and the engine has to relight.
%   velocityDrift           0.1 m/s^2 drift of the measured vx: the
%                           lateral law steers the stage off the pad, and
%                           the observer, which differentiates the same
%                           measurement, adds to the miss.
%
%   Model interface expected by the tests: as in TestLanding, with
%     - the thrust applied to the plant (root Outport 9) multiplied by
%       1 - p.fail.dT from time p.fail.tT on;
%     - the guidance reading z + p.fail.zBias instead of z;
%     - lateral_ctrl and the observer reading vx + p.fail.vxDrift*t
%       instead of vx.
%
%   Run from the repository root:  runtests('tests')

    properties (Constant)
        Model = 'landing'
        % 10 s after the nominal ignition at 4.9 s
        tFail = 15
    end

    methods (TestClassSetup)
        function addPaths(tc)
            root = fileparts(fileparts(mfilename('fullpath')));
            tc.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            tc.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, 'models')));
        end
    end

    methods (Test)
        function thrustLossWithinMargin(tc)
            % Reference model: touchdown at 1.2 m/s.
            p = params();
            p.fail.dT = 0.15;  p.fail.tT = tc.tFail;
            s = tc.simulate(p);

            tc.assertLessThan(s.t(end), 120, 'No touchdown within 120 s.');
            tc.verifyLessThan(abs(s.vz(end)),    p.req.vz);
            tc.verifyLessThan(abs(s.vx(end)),    p.req.vx);
            tc.verifyLessThan(abs(s.theta(end)), p.req.theta);
            tc.verifyLessThan(abs(s.x(end)),     p.req.x);
            tc.verifyGreaterThan(s.m(end), p.mdry, 'Out of propellant.');
            tc.verifyEqual(sum(diff(s.T > 0) > 0), 1, 'The engine must light exactly once.');
            tc.verifyThrustCapped(p, s);
        end

        function thrustLossBeyondMargin(tc)
            % Reference model: impact at 53 m/s.
            p = params();
            p.fail.dT = 0.30;  p.fail.tT = tc.tFail;
            s = tc.simulate(p);

            tc.verifyGreaterThan(abs(s.vz(end)), 5*p.req.vz);
            tc.verifyThrustCapped(p, s);
        end

        function altimeterReadsHigh(tc)
            % Reference model: impact at 3.34 m/s against 3.32 m/s from
            % the closed form (4.92 m/s without the gate).
            p = params();
            p.fail.zBias = 1;
            s = tc.simulate(p);
            tc.assertLessThan(s.t(end), 120, 'No touchdown within 120 s.');
            tc.verifyEqual(abs(s.vz(end)), sqrt(p.vTouch^2 + 2*p.aGate*1), 'RelTol', 0.05);
        end

        function gateToleratesBias(tc)
            % Reference model: 1.74 m/s with a 0.2 m bias, 2.01 at 0.3 m.
            p = params();
            p.fail.zBias = 0.25;
            s = tc.simulate(p);
            tc.assertLessThan(s.t(end), 120, 'No touchdown within 120 s.');
            tc.verifyLessThan(abs(s.vz(end)), p.req.vz);
            tc.verifyEqual(sum(diff(s.T > 0) > 0), 1, 'The engine must light exactly once.');
        end

        function altimeterReadsLow(tc)
            % Reference model: 44 ignitions, touchdown at 1.2 m/s.
            p = params();
            p.fail.zBias = -10;
            s = tc.simulate(p);
            tc.assertLessThan(s.t(end), 120, 'No touchdown within 120 s.');
            tc.verifyGreaterThan(sum(diff(s.T > 0) > 0), 1, 'The engine should have relit.');
        end

        function velocityDrift(tc)
            % Reference model: miss 18.6 m with the observer, 16.0 m without.
            p = params();
            p.fail.vxDrift = 0.1;
            on = tc.simulate(p);
            p.kDob = 0;
            off = tc.simulate(p);
            tc.verifyGreaterThan(abs(on.x(end)), p.req.x);
            tc.verifyGreaterThan(abs(off.x(end)), p.req.x);
            % the observer differentiates the drifting measurement too
            tc.verifyGreaterThan(abs(on.x(end)), abs(off.x(end)) + 1);
        end
    end

    methods (Access = private)
        function verifyThrustCapped(tc, p, s)
            % After the failure the applied thrust cannot exceed the
            % remaining fraction of Tmax; before it, Tmax itself.
            after = s.t > p.fail.tT + 1e-6;
            tc.verifyLessThanOrEqual(max(s.T(after)), (1 - p.fail.dT)*p.kT*p.Tmax*(1 + 1e-9));
            tc.verifyLessThanOrEqual(max(s.T(~after)), p.kT*p.Tmax*(1 + 1e-9));
        end

        function s = simulate(tc, p)
            in = Simulink.SimulationInput(tc.Model);
            in = in.setVariable('p', p);
            in = in.setModelParameter('Solver', 'ode45', ...
                'RelTol', '1e-8', 'AbsTol', '1e-8', 'StopTime', '120', ...
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
