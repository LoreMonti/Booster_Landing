# Roadmap

The project was built one reviewable step at a time, following a written
plan of five steps and an optional sixth: each step ended with its tests passing
and a commit. The physics and the method behind every item are in the
[README](README.md); this file records what was done, in what order, and what
each step taught. The Simulink models were built by hand in the editor, one
block at a time; the parameters, tests and studies were written around them.

## Step 0 — Tools

- [x] MATLAB R2026b with Simulink, the Control System Toolbox and the Statistics and Machine Learning Toolbox, on the university campus licence
- [x] Simulink Onramp completed, course certificate obtained
- [x] Headless runs from the terminal (`matlab -batch`), so tests and studies run without the editor

*Lesson.* The Onramp would not start because macOS had cached a failed DNS
lookup of the course server, and the progress bar stayed at 97 % until the
page was reloaded: when a tool fails, check the network before the tool.

## Step 1 — The plant

- [x] Vertical channel `plant_vertical.slx`: $`\ddot z = T/m - g`$, $`\dot m = -T/(I_{sp} g_0)`$, three integrators
- [x] Every parameter in `params.m`, none in the blocks
- [x] Falcon 9-class landing parameters from public sources; ignition mass, lever arm, inertia and drag coefficient estimated and documented
- [x] Planar 3-DOF plant `plant_3dof.slx`: seven integrators, the derivatives in one MATLAB Function, engine cut at dry mass, stop at touchdown
- [x] Feedback through Goto/From tags, one row per equation
- [x] Tests: free fall, constant thrust, reduction to the vertical channel, tilted thrust, constant gimbal torque, drag-limited fall
- [x] `run_all.m`, the validation figure and the open-loop descents

*Lesson.* The plan itself had a sign error: drag was defined with a minus
sign and then subtracted, so it would have pushed the stage along its
velocity. In the model, an extra Divide block made $`\dot m = T/0`$ and an
unquoted state name was read as a variable; dividing by $`l`$ instead of $`I`$
would have given a pitch acceleration 300 000 times too large with no error
message. Each of these is now a test, and the tests were checked by putting
the bugs back into a reference model and watching them fail.

## Step 2 — Attitude loop

- [x] Plant inputs $`T`$ and $`\delta`$ as Inports, the plant referenced by a Model block
- [x] PD on the commanded pitch acceleration, inverted through $`\delta = -\ddot\theta_{cmd} I/(T l)`$
- [x] Gimbal as a first-order servo with angle and rate limits
- [x] Controller moved into the Subsystem Reference `attitude_ctrl.slx`, shared by every model
- [x] Tests: tilt recovery at full and minimum thrust along the ideal response, limits on a saturating step, phase margin
- [x] Margins and time responses: 56 deg, delay margin 0.64 s

*Lesson.* Simulink's Rate Limiter passes its first input through in
continuous time, so the gimbal jumped to its limit at $`t = 0`$; the servo is
now an integrator, which starts where it should. Referencing the plant raised
a false algebraic loop until the plant was set to minimise them. A PD was
kept instead of the PID of the plan: the model has no constant pitch disturbance
for an integral to reject.

## Step 3 — Vertical guidance

- [x] Closed-form ignition altitude of the full-thrust hoverslam: 2180 m, 15.0 s from 280 m/s
- [x] Constant-deceleration law $`a_{req} = (u^2 - u_t^2)/(2z)`$, ignition by a relay with hysteresis at $`\eta = 0.7`$ of the maximum deceleration
- [x] `landing.slx`: guidance, attitude controller and plant
- [x] Drag feedforward and integral term on $`a_{req} - a^*`$
- [x] Tests: hoverslam altitude against the plant, every requirement under seven unknown errors, a single ignition, nominal efficiency
- [x] Landing study: 1.20 m/s at touchdown, 25 s burn, 1686 kg left

*Lesson.* The first guidance landed safely and wasted a ton of propellant:
drag over-braked the stage, so the required deceleration kept shrinking and
the burn stretched to 34 s. The giveaway was that a weaker engine landed with
more propellant. Subtracting the predicted drag fixed it and exposed a
weakness the waste had hidden: with a thrust 5 % low, the required
deceleration grows like $`u/z`$ near the ground and the stage landed at
2.4 m/s. An integral term fixed that. With $`\eta = 1`$, the fuel-optimal
choice, a 3 % error crashed at 75 m/s.

## Step 4 — Lateral control, PD against LQR

- [x] Lateral and pitch dynamics as a chain of four integrators after the inversion
- [x] One `lateral_ctrl` block, $`\theta_{ref} = -(K_1 x + K_2 \dot x)/a + K_3 \theta + K_4 q`$, flown with two gain sets
- [x] Cascaded PD and LQR tuned for the same peak effort
- [x] Tests: the uncontrolled offset misses by more than 50 m, both gain sets meet every requirement, nominally and with combined errors
- [x] Comparison: LQR 0.004–0.19 m from the pad against 0.4–1.6 m, 5–10 s sooner

*Lesson.* The step as written, LQR against PID on pitch alone, would have
compared a PD with a PD: the LQR of a double integrator has that form. The
comparison only means something on the coupled lateral and pitch states. The
first LQR, with textbook weights, asked for 25 times the gimbal authority, so
controllers are compared at equal effort.

## Step 5 — Monte Carlo

- [x] Constant horizontal wind in the plant: drag on the velocity relative to the air, with a closed-form test
- [x] Gaussian dispersions of thrust, Isp, mass, drag, wind and initial state in `params.m`, seed fixed
- [x] 1000 landings in about 3 minutes: 100 % success, 95 % interval 99.6–100 %, 99th percentile miss 3.8 m
- [x] Convergence of the success rate and of the spread with the number of runs
- [x] Stress test with dispersions scaled 1 to 4 times: 100, 99, 95, 83, 80, 57 %

*Lesson.* A 100 % success rate says how often the design works, not how far
it is from failing. Scaling the dispersions until landings failed showed the
weak point: the lateral speed at touchdown, driven by wind. The first reading
of why, a constant wind leaving a residual drift that an integral would
remove, was wrong; extension 2 corrects it.

## Extension 2 — Lateral disturbance observer

- [x] Diagnosis: the wind force follows the airspeed squared, about 0.7 m/s² early in the burn and nearly zero at the ground; with a 25 m/s wind the stage lands 7.3 m off still moving at 0.9 m/s, not settled at an offset
- [x] Lateral integral tried on a copy of the model and rejected: it winds up during the saturated phase (33 m miss at $`K_I = 0.005`$, 105 m at 0.02); integrating only near the pad does nothing against the wind
- [x] Observer $`\hat d = [\ddot x - (T_{cmd}/m)\sin\theta]/(\tau_d s + 1)`$ built in `landing.slx`, cancelled in `lateral_ctrl` through $`-k_d\hat d/a`$
- [x] Tests: with a 15 m/s wind, thrust 5 % low and $`C_D = 1.2`$ the stage fails the lateral speed with the observer off and meets every requirement with it on
- [x] Tuning: $`k_d = 1`$, $`\tau_d = 0.5`$ s, with sweeps of both on the test case and $`k_d = 0.75`$ against 1 on the dispersed landings
- [x] Studies rerun: 99th-percentile miss 3.8 to 1.9 m; stress test 100, 99.7, 98, 94, 87, 71 %; lateral-speed failures at 2.5× from 41 to 7 of 300

*Lesson.* The integral was the textbook answer to a disturbance that was
assumed constant without checking; plotting the disturbance against time
showed it decaying by an order of magnitude during the burn, and the test
cases showed the integral making things worse. On the test case the observer
overshoots the pad by 4.6 m, because its estimate lags a wind force that is
decaying; a fraction $`k_d = 0.75`$ maximised the worst margin on that one case,
but on the dispersed landings it was no better than 1, and a gain tuned on one
case is tuned to that case's wind.

## Extension 3 — Trajectory animation

- [x] `study_animation.m`: one landing (30 m, 2 m/s, 10 m/s wind) flown in `landing.slx` and drawn frame by frame
- [x] Two views: the whole descent with the horizontal scale stretched, and a 90 m window that follows the stage at true scale with tilt, gimbal-directed plume, legs and telemetry
- [x] GIF at twice real time, one shared colour map, at the top of the README

*Lesson.* The descent is 5 km tall and under 100 m wide while the stage is
42 m long, so no single view at true scale shows both the path and the
attitude; splitting the picture into a stretched overview and a true-scale
close-up solved it. The first version drew the plume through the ground and
kept the engine lit in the touchdown frame.

## Possible extensions

- [ ] **Navigation errors.** The controllers see the true state; sensor noise and a Kalman filter would set the filter of the disturbance observer, which differentiates a velocity, and show whether it and the guidance integral tolerate biased measurements.
- [ ] **Failure injection and divert** (the optional step 6 of the plan): a sudden thrust loss or a sensor bias, and a retarget to a safe point.
- [ ] **Throttle dynamics.** A first-order engine response would show how much of the 20 % throttle margin the guidance really has.
- [ ] **Convex-optimisation guidance** (README reference 5): the fuel-optimal landing as a benchmark for the feedback law, to measure what $`\eta = 0.7`$ costs.
- [ ] **Out-of-plane motion.** A second lateral axis would give a true landing ellipse on the ground.
- [ ] **Aerodynamics with angle of attack.** A pitching moment and a drag growing with the tilt, the largest simplification in the model.
- [ ] **Varying inertia.** Moment of inertia and centre of mass changing as propellant burns, which the inversion would then have to track.
