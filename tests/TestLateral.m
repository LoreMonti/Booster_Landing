classdef TestLateral < matlab.unittest.TestCase
%TESTLATERAL Checks the lateral control of models/landing.slx (step 4).
%
%   The stage starts 30 m off the pad and drifting at 2 m/s. Without
%   lateral control it misses the pad by more than 70 m; with either gain
%   set of params.m (cascaded PD or LQR) it must meet every touchdown
%   requirement, nominally and with the combined thrust, drag and Isp
%   error of TestLanding, with the commanded tilt within its limit and a
%   single ignition.
%
%   Wind (extension 2): the same offset with a 15 m/s wind, a thrust 5 %
%   low and CD = 1.2. With the proportional loop alone (p.kDob = 0) the
%   wind drag pushes the stage off course until touchdown and it lands
%   still drifting faster than p.req.vx; with the disturbance observer
%   (p.kDob, p.tauDob of params.m) it must meet every requirement.
%
%   Model interface expected by the tests: as in TestLanding, with a
%   lateral_ctrl block computing thetaRef from x, vx, theta, q, the
%   commanded thrust, m and the disturbance estimate dHat, using the gains
%   p.Klat and the observer parameters p.kDob, p.tauDob.
%
%   Run from the repository root:  runtests('tests')

    properties (Constant)
        Model = 'landing'
    end

    properties (TestParameter)
        % Reference model: PD misses by 0.9-1.5 m, LQR by 0.02-0.16 m.
        gains = {'KlatPD', 'KlatLQR'}
        dispersion = struct( ...
            'nominal',  {{}}, ...
            'combined', {{'kT', 0.95, 'CD', 0.4, 'Isp', 0.95*282}})
    end

    methods (TestClassSetup)
        function addPaths(tc)
            root = fileparts(fileparts(mfilename('fullpath')));
            tc.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            tc.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, 'models')));
        end
    end

    methods (Test)
        function offsetNeedsLateralControl(tc)
            % The scenario is meaningful: with no lateral control the
            % stage lands far outside the 10 m requirement (reference: 71 m).
            p = TestLateral.offset(params());
            p.Klat = [0 0 0 0];
            s = tc.simulate(p);
            tc.verifyGreaterThan(abs(s.x(end)), 5*p.req.x);
        end

        function lateralLanding(tc, gains, dispersion)
            p = TestLateral.offset(params());
            p.Klat = p.(gains);
            for j = 1:2:numel(dispersion)
                p.(dispersion{j}) = dispersion{j + 1};
            end
            tc.verifyTouchdown(p, tc.simulate(p));
        end

        function windNeedsObserver(tc)
            % The wind case is meaningful: without the observer the stage
            % touches down drifting at more than p.req.vx (reference
            % model: x = 7.6 m, vx = -1.46 m/s).
            p = TestLateral.wind(TestLateral.offset(params()));
            p.kDob = 0;
            s = tc.simulate(p);
            tc.assertLessThan(s.t(end), 120, 'No touchdown within 120 s.');
            tc.verifyGreaterThan(abs(s.vx(end)), p.req.vx);
        end

        function windRejection(tc)
            % Reference model with the observer: x = -4.6 m, vx = -0.02 m/s.
            p = TestLateral.wind(TestLateral.offset(params()));
            tc.verifyTouchdown(p, tc.simulate(p));
        end
    end

    methods (Static, Access = private)
        function p = offset(p)
            % 30 m off the pad, drifting away at 2 m/s, at the start of
            % the terminal phase.
            p.x0 = 30;
            p.vx0 = 2;
        end

        function p = wind(p)
            % 15 m/s wind towards +x, thrust 5 % low, drag 50 % high:
            % the guidance and the lateral law know none of them.
            p.wind = 15;
            p.kT = 0.95;
            p.CD = 1.2;
        end
    end

    methods (Access = private)
        function verifyTouchdown(tc, p, s)
            tc.assertLessThan(s.t(end), 120, 'No touchdown within 120 s.');
            tc.verifyLessThan(abs(s.x(end)),     p.req.x);
            tc.verifyLessThan(abs(s.vx(end)),    p.req.vx);
            tc.verifyLessThan(abs(s.vz(end)),    p.req.vz);
            tc.verifyLessThan(abs(s.theta(end)), p.req.theta);
            % the tilt follows a reference limited to thetaRefMax
            tc.verifyLessThan(max(abs(s.theta)), p.thetaRefMax + deg2rad(1));
            tc.verifyLessThanOrEqual(max(abs(s.delta)), p.deltaMax*(1 + 1e-9));
            tc.verifyEqual(sum(diff(s.T > 0) > 0), 1, 'The engine must light exactly once.');
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
