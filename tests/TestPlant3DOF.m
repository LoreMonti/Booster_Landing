classdef TestPlant3DOF < matlab.unittest.TestCase
%TESTPLANT3DOF Checks models/plant_3dof.slx term by term against closed forms.
%
%   The full planar 3-DOF model has no closed-form solution, but each of
%   its terms does once the others are switched off:
%     reduction       theta = delta = 0, no drag  -> the vertical channel
%     tiltedThrust    fixed theta, g = 0, no drag -> rocket equation along theta
%     gimbalTorque    constant T and delta        -> theta quadratic in t
%     drag            T = 0, constant density     -> tanh approach to terminal velocity
%   Each test catches a specific silent error: sin and cos swapped, wrong
%   torque sign, missing 1/2 or a drag that pushes instead of brakes.
%
%   Model interface expected by the tests:
%     - parameters read from the struct p (see params.m);
%     - root Inports 1-2 = thrust T [N] and gimbal angle delta [rad], fed
%       here with the constant commands p.T and p.delta;
%     - root Outports 1-7 = x, z, vx, vz, theta, q, m (SI units, rad).
%
%   Run from the repository root:  runtests('tests')

    properties (Constant)
        Model = 'plant_3dof'
        RelTol = 1e-10      % solver tolerances forced by the tests
        AbsTol = 1e-10
    end

    methods (TestClassSetup)
        function addPaths(tc)
            root = fileparts(fileparts(mfilename('fullpath')));
            tc.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            tc.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, 'models')));
        end
    end

    methods (Test)
        function reduction(tc)
            % With the vehicle upright and no drag the 3-DOF model must
            % reproduce the vertical channel and leave x, theta untouched.
            p = params();
            p.CD = 0;
            tEnd = 15;
            s = tc.simulate(p, tEnd);

            [mRef, L, k, c] = TestPlant3DOF.rocket(p, s.t);
            tc.verifyEqual(s.m,  mRef, 'RelTol', 1e-12);
            tc.verifyEqual(s.vz, p.vz0 - p.g*s.t + c*L, 'AbsTol', 1e-8);
            tc.verifyEqual(s.z,  p.z0 + p.vz0*s.t - 0.5*p.g*s.t.^2 + c*(s.t - (mRef/k).*L), ...
                'AbsTol', 1e-8);
            tc.verifyEqual(s.x,     zeros(size(s.t)), 'AbsTol', 1e-12);
            tc.verifyEqual(s.theta, zeros(size(s.t)), 'AbsTol', 1e-12);
        end

        function tiltedThrust(tc)
            % theta fixed at theta0 (q0 = 0, delta = 0), no gravity, no
            % drag: the rocket-equation velocity points along theta, so
            % vx gets sin(theta0) and vz gets cos(theta0).
            p = params();
            p.g = 0;  p.CD = 0;  p.theta0 = deg2rad(10);
            tEnd = 15;
            s = tc.simulate(p, tEnd);

            [mRef, L, k, c] = TestPlant3DOF.rocket(p, s.t);
            S = c*(s.t - (mRef/k).*L);         % integral of c*ln(m0/m) dt
            tc.verifyEqual(s.vx, p.vx0 + c*L*sin(p.theta0), 'AbsTol', 1e-8);
            tc.verifyEqual(s.vz, p.vz0 + c*L*cos(p.theta0), 'AbsTol', 1e-8);
            tc.verifyEqual(s.x,  p.x0 + p.vx0*s.t + S*sin(p.theta0), 'AbsTol', 1e-8);
            tc.verifyEqual(s.z,  p.z0 + p.vz0*s.t + S*cos(p.theta0), 'AbsTol', 1e-8);
            tc.verifyEqual(s.theta, p.theta0*ones(size(s.t)), 'AbsTol', 1e-12);
        end

        function gimbalTorque(tc)
            % Constant T and delta give a constant pitch acceleration
            % -T*l*sin(delta)/I: theta is quadratic in t (round-off level).
            % delta > 0 must pitch the vehicle towards -x.
            p = params();
            p.delta = deg2rad(1);
            tEnd = 5;
            s = tc.simulate(p, tEnd);

            alpha = -p.T*p.l*sin(p.delta)/p.I;
            tc.verifyEqual(s.q,     p.q0 + alpha*s.t, 'AbsTol', 1e-10);
            tc.verifyEqual(s.theta, p.theta0 + p.q0*s.t + 0.5*alpha*s.t.^2, 'AbsTol', 1e-10);
            tc.verifyLessThan(s.theta(end), 0);
        end

        function drag(tc)
            % T = 0, upright, constant density (H = Inf): zddot = -g + k*vz^2
            % for vz < 0, so vz tends to -vt with vt = sqrt(g/k) and
            %   vz(t) = -vt*tanh(g*t/vt + atanh(|vz0|/vt)).
            % A drag with the wrong sign never approaches vt.
            p = params();
            p.T = 0;  p.H = Inf;  p.vz0 = -100;
            tEnd = 10;
            s = tc.simulate(p, tEnd);

            kd = p.rho0*p.CD*p.A/(2*p.m0);
            vt = sqrt(p.g/kd);
            tc.assertLessThan(abs(p.vz0), vt);
            a  = atanh(abs(p.vz0)/vt);
            tc.verifyEqual(s.vz, -vt*tanh(p.g*s.t/vt + a), 'AbsTol', 1e-8);
            tc.verifyEqual(s.z,  p.z0 - vt^2/p.g*(log(cosh(p.g*s.t/vt + a)) - log(cosh(a))), ...
                'AbsTol', 1e-8);
            tc.verifyEqual(s.m,  p.m0*ones(size(s.t)), 'AbsTol', 1e-9);
        end
    end

    methods (Static, Access = private)
        function [mRef, L, k, c] = rocket(p, t)
            % Mass and log term of the rocket equation at constant thrust.
            k    = p.T/(p.Isp*p.g0);           % propellant mass flow [kg/s]
            c    = p.Isp*p.g0;                 % effective exhaust velocity [m/s]
            mRef = p.m0 - k*t;
            L    = log(p.m0./mRef);
        end
    end

    methods (Access = private)
        function s = simulate(tc, p, tEnd)
            in = Simulink.SimulationInput(tc.Model);
            in = in.setVariable('p', p);
            in = in.setExternalInput([0 p.T p.delta; tEnd p.T p.delta]);
            in = in.setModelParameter('Solver', 'ode45', ...
                'RelTol', num2str(tc.RelTol), 'AbsTol', num2str(tc.AbsTol), ...
                'StopTime', num2str(tEnd), 'SaveOutput', 'on', ...
                'OutputSaveName', 'yout', 'SaveFormat', 'Dataset');
            out = sim(in);
            tc.assertEmpty(out.ErrorMessage);

            names = {'x', 'z', 'vx', 'vz', 'theta', 'q', 'm'};
            for i = 1:numel(names)
                s.(names{i}) = out.yout{i}.Values.Data;
            end
            s.t = out.yout{1}.Values.Time;
            % A touchdown or a burnout would stop or change the run early.
            tc.assertEqual(s.t(end), tEnd, 'AbsTol', 1e-9, ...
                'Simulation ended before tEnd: check the ground stop.');
            tc.assertGreaterThan(min(s.m), p.mdry);
        end
    end
end
