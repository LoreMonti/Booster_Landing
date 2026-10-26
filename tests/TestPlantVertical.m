classdef TestPlantVertical < matlab.unittest.TestCase
%TESTPLANTVERTICAL Checks models/plant_vertical.slx against closed-form solutions.
%
%   The vertical channel with theta = delta = 0 and no drag,
%       zddot = T/m - g,    mdot = -T/(Isp*g0),
%   has exact solutions for zero and for constant thrust. A wrong sign, a
%   missing factor or a feedback loop wired to the wrong signal shows up
%   as an error many orders of magnitude above the tolerances used here.
%
%   Model interface expected by the tests:
%     - parameters read from the struct p (see params.m), thrust from p.T;
%     - root Outports 1, 2, 3 = z [m], vz [m/s], m [kg].
%
%   Run from the repository root:  runtests('tests')

    properties (Constant)
        Model = 'plant_vertical'
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
        function freeFall(tc)
            % T = 0: z is a quadratic in t, which ode45 integrates exactly,
            % so only round-off is left (measured with the params.m values:
            % max error 6e-12 m on z).
            p = params();
            p.T = 0;
            [t, z, vz, m] = tc.simulate(p, 10);

            tc.verifyEqual(z,  p.z0 + p.vz0*t - 0.5*p.g*t.^2, 'AbsTol', 1e-9);
            tc.verifyEqual(vz, p.vz0 - p.g*t,                  'AbsTol', 1e-9);
            tc.verifyEqual(m,  p.m0*ones(size(t)),             'AbsTol', 1e-9);
        end

        function constantThrust(tc)
            % Constant T: mass falls linearly and vz follows the rocket
            % equation with gravity. The log makes the error depend on the
            % solver tolerance, not on round-off (measured with the params.m
            % values and RelTol = AbsTol = 1e-10: max error 5e-11 m on z).
            p = params();
            k = p.T/(p.Isp*p.g0);              % propellant mass flow [kg/s]
            tEnd = 15;
            tc.assumeGreaterThan(p.m0 - k*tEnd, p.mdry, ...
                'Burn would go below dry mass: shorten tEnd.');
            [t, z, vz, m] = tc.simulate(p, tEnd);

            c = p.Isp*p.g0;                    % effective exhaust velocity [m/s]
            mRef  = p.m0 - k*t;
            L     = log(p.m0./mRef);
            vzRef = p.vz0 - p.g*t + c*L;
            zRef  = p.z0 + p.vz0*t - 0.5*p.g*t.^2 + c*(t - (mRef/k).*L);

            tc.verifyEqual(m,  mRef,  'RelTol', 1e-12);
            tc.verifyEqual(vz, vzRef, 'AbsTol', 1e-8);
            tc.verifyEqual(z,  zRef,  'AbsTol', 1e-8);
        end
    end

    methods (Access = private)
        function [t, z, vz, m] = simulate(tc, p, tEnd)
            in = Simulink.SimulationInput(tc.Model);
            in = in.setVariable('p', p);
            in = in.setModelParameter('Solver', 'ode45', ...
                'RelTol', num2str(tc.RelTol), 'AbsTol', num2str(tc.AbsTol), ...
                'StopTime', num2str(tEnd), 'SaveOutput', 'on', ...
                'OutputSaveName', 'yout', 'SaveFormat', 'Dataset');
            out = sim(in);
            tc.assertEmpty(out.ErrorMessage);

            z  = out.yout{1}.Values;
            vz = out.yout{2}.Values;
            m  = out.yout{3}.Values;
            t  = z.Time;
            z  = z.Data;  vz = vz.Data;  m = m.Data;
        end
    end
end
