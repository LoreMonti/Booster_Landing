classdef TestAttitudeLoop < matlab.unittest.TestCase
%TESTATTITUDELOOP Checks models/attitude_loop.slx: the pitch PD with dynamic inversion.
%
%   The controller commands thetaddot_cmd = Kp*(thetaRef - theta) - Kd*q
%   and inverts the plant with delta_cmd = -thetaddot_cmd*I/(T*l). The
%   gimbal is a first-order servo (lag tauAct) with rate and position
%   limits. The tests check that:
%     - a 3 deg tilt is recovered at full and at minimum thrust, with the
%       same response, i.e. the inversion removes the throttle dependence;
%     - the gimbal starts at 0 and never exceeds its angle and rate limits,
%       also on a 10 deg step that saturates it;
%     - the linearised loop has a phase margin above 45 deg.
%
%   Model interface expected by the tests:
%     - parameters read from the struct p (see params.m), constant thrust
%       p.T and pitch reference p.thetaRef;
%     - plant_3dof referenced through a Model block;
%     - root Outports 1-7 = x, z, vx, vz, theta, q, m and 8 = delta.
%
%   Run from the repository root:  runtests('tests')

    properties (Constant)
        Model = 'attitude_loop'
    end

    properties (TestParameter)
        thrust = {'Tmax', 'Tmin'}
    end

    methods (TestClassSetup)
        function addPaths(tc)
            root = fileparts(fileparts(mfilename('fullpath')));
            tc.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            tc.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, 'models')));
        end
    end

    methods (Test)
        function tiltRecovery(tc, thrust)
            % A 3 deg initial tilt must be back within 0.5 deg in 8 s and
            % follow the ideal second-order response s^2 + Kd*s + Kp up to
            % the gimbal lag (measured on a reference model: 0.07 deg at
            % Tmax, 0.13 deg at Tmin).
            p = params();
            p.T = p.(thrust);
            p.theta0 = deg2rad(3);
            s = tc.simulate(p, 10);

            tc.verifyLessThan(max(abs(s.theta(s.t >= 8))), deg2rad(0.5));
            tc.verifyEqual(s.theta, TestAttitudeLoop.ideal(p, s.t), 'AbsTol', deg2rad(0.2));
            tc.verifyGimbalLimits(p, s);
        end

        function saturatedStep(tc)
            % A 10 deg reference at minimum thrust asks for more pitch
            % acceleration than the gimbal can give: the limits must hold
            % and the loop must still settle (reference model: 3.1 s).
            p = params();
            p.T = p.Tmin;
            p.thetaRef = deg2rad(10);
            s = tc.simulate(p, 10);

            tc.verifyGreaterThan(max(abs(s.delta)), 0.95*p.deltaMax);   % saturation reached
            tc.verifyGimbalLimits(p, s);
            tc.verifyLessThan(max(abs(s.theta(s.t >= 5) - p.thetaRef)), deg2rad(0.5));
        end

        function phaseMargin(tc)
            % After the inversion the open loop is (Kd*s + Kp)/s^2 times
            % the gimbal lag 1/(tauAct*s + 1), at every throttle setting.
            p = params();
            s = tf('s');
            L = (p.KdAtt*s + p.KpAtt)/s^2/(p.tauAct*s + 1);
            [~, pm] = margin(L);
            tc.verifyGreaterThan(pm, 45);
        end
    end

    methods (Access = private)
        function verifyGimbalLimits(tc, p, s)
            % The gimbal starts centred and respects angle and rate limits.
            tc.verifyEqual(s.delta(1), 0, 'AbsTol', 1e-12);
            tc.verifyLessThanOrEqual(max(abs(s.delta)), p.deltaMax*(1 + 1e-9));
            % Finite differences over very short solver steps amplify
            % round-off, so the rate is measured on steps of at least 0.1 ms.
            dt = diff(s.t);  ok = dt >= 1e-4;
            rate = diff(s.delta)./dt;
            tc.verifyLessThanOrEqual(max(abs(rate(ok))), p.deltaRate*(1 + 1e-3));
        end

        function s = simulate(tc, p, tEnd)
            in = Simulink.SimulationInput(tc.Model);
            in = in.setVariable('p', p);
            in = in.setModelParameter('Solver', 'ode45', ...
                'RelTol', '1e-8', 'AbsTol', '1e-8', 'StopTime', num2str(tEnd), ...
                'SaveOutput', 'on', 'OutputSaveName', 'yout', 'SaveFormat', 'Dataset');
            out = sim(in);
            tc.assertEmpty(out.ErrorMessage);

            names = {'x', 'z', 'vx', 'vz', 'theta', 'q', 'm', 'delta'};
            for i = 1:numel(names)
                s.(names{i}) = out.yout{i}.Values.Data;
            end
            s.t = out.yout{1}.Values.Time;
            tc.assertEqual(s.t(end), tEnd, 'AbsTol', 1e-9, ...
                'Simulation ended before tEnd: check the ground stop.');
        end
    end

    methods (Static, Access = private)
        function th = ideal(p, t)
            % Free response of s^2 + Kd*s + Kp from theta0, q0 = 0 (zeta < 1).
            z  = p.zetaAtt;  w = p.wnAtt;  wd = w*sqrt(1 - z^2);
            th = p.thetaRef + (p.theta0 - p.thetaRef)*exp(-z*w*t) ...
                .*(cos(wd*t) + z/sqrt(1 - z^2)*sin(wd*t));
        end
    end
end
